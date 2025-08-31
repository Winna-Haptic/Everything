/*
 * Location-Specific Haptic Controller
 * Precise haptic feedback mapped to biomechanical zones
 * 
 * Core Principle: User feels vibration exactly where their form is CORRECT
 */

#ifndef LOCATION_HAPTIC_CONTROLLER_H
#define LOCATION_HAPTIC_CONTROLLER_H

#include "architecture.h"
#include <array>
#include <queue>
#include <chrono>

class LocationSpecificHapticController : public HapticController {
private:
    // Hardware mapping (customize based on actual haptic motor placement)
    struct HapticMotor {
        uint8_t pin;              // GPIO pin or I2C address
        HapticZone zone;          // Which biomechanical zone this motor covers
        double current_intensity; // 0.0 to 1.0
        uint64_t last_trigger_us; // Prevent over-triggering
        bool is_active;
        uint32_t pattern_id;      // Current waveform
        
        // Motor characteristics
        double max_intensity;     // Hardware limit
        uint32_t min_pulse_ms;    // Minimum meaningful pulse
        uint32_t max_pulse_ms;    // Maximum safe pulse
        double resonant_freq;     // Optimal frequency for this motor
        
        HapticMotor() : pin(0), zone(MAX_HAPTIC_ZONES), current_intensity(0.0),
                       last_trigger_us(0), is_active(false), pattern_id(0),
                       max_intensity(1.0), min_pulse_ms(50), max_pulse_ms(500),
                       resonant_freq(235.0) {}
    };
    
    std::array<HapticMotor, MAX_HAPTIC_ZONES> motors_;
    
    // Command queue for precise timing
    std::queue<HapticCommand> command_queue_;
    
    // System state
    bool system_enabled_;
    double global_intensity_scale_;
    uint64_t last_update_us_;
    
    // Safety limits
    static constexpr uint32_t MIN_INTER_PULSE_MS = 100;  // Prevent rapid-fire
    static constexpr uint32_t MAX_SIMULTANEOUS_MOTORS = 3; // Prevent overwhelming user
    static constexpr double MAX_TOTAL_INTENSITY = 2.0;   // Total intensity across all motors
    
    // Waveform patterns (can be expanded)
    enum WaveformPattern {
        PATTERN_SINGLE_PULSE = 1,
        PATTERN_DOUBLE_PULSE = 2,
        PATTERN_SUCCESS_BURST = 3,
        PATTERN_GENTLE_FADE = 4,
        PATTERN_PRECISION_CLICK = 5
    };

public:
    LocationSpecificHapticController() : system_enabled_(true), global_intensity_scale_(1.0), last_update_us_(0) {
        initializeMotorMapping();
    }
    
    // HapticController interface
    void initialize() override;
    void triggerFeedback(const HapticCommand& command) override;
    void stopAll() override;
    bool isZoneActive(HapticZone zone) override;
    void setIntensityScale(double scale) override;
    
    // Location-specific methods
    void mapMotorToZone(uint8_t pin, HapticZone zone, double resonant_freq = 235.0);
    void triggerLocationSpecificFeedback(const std::vector<HapticCommand>& commands);
    void calibrateMotorIntensity(HapticZone zone, double optimal_intensity);
    
    // Advanced control
    void setMotorCharacteristics(HapticZone zone, double max_intensity, uint32_t min_pulse, uint32_t max_pulse);
    void updateMotorResonance(HapticZone zone, double frequency);
    
    // Real-time processing
    void processCommandQueue();
    void updateActiveMotors();
    
    // Safety and validation
    bool validateCommand(const HapticCommand& command);
    void enforceIntensityLimits();
    void preventOverstimulation();
    
private:
    void initializeMotorMapping();
    void executeWaveform(HapticMotor& motor, WaveformPattern pattern, double intensity, uint32_t duration_ms);
    void hardwareSetIntensity(uint8_t pin, double intensity);
    void hardwareSetFrequency(uint8_t pin, double frequency);
    double calculateOptimalIntensity(const HapticCommand& command);
    uint32_t calculateOptimalDuration(const HapticCommand& command);
    
    // Hardware abstraction (implement based on your hardware)
    void initializeHardware();
    void sendPWMSignal(uint8_t pin, double duty_cycle);
    void sendI2CCommand(uint8_t address, uint8_t command, uint8_t data);
};

// ===== IMPLEMENTATION =====

inline void LocationSpecificHapticController::initialize() {
    initializeHardware();
    initializeMotorMapping();
    
    // Clear all motors
    stopAll();
    
    system_enabled_ = true;
    global_intensity_scale_ = 1.0;
    last_update_us_ = getCurrentTimeMicros();
}

inline void LocationSpecificHapticController::initializeMotorMapping() {
    // Default motor-to-zone mapping (customize for your hardware)
    
    // Finger motors (assume small coin motors on fingertips)
    motors_[THUMB_ZONE] = {.pin = 2, .zone = THUMB_ZONE, .resonant_freq = 200.0};
    motors_[INDEX_ZONE] = {.pin = 3, .zone = INDEX_ZONE, .resonant_freq = 235.0};
    motors_[MIDDLE_ZONE] = {.pin = 4, .zone = MIDDLE_ZONE, .resonant_freq = 235.0};
    motors_[RING_ZONE] = {.pin = 5, .zone = RING_ZONE, .resonant_freq = 200.0};
    motors_[PINKY_ZONE] = {.pin = 6, .zone = PINKY_ZONE, .resonant_freq = 180.0};
    
    // Wrist motors (assume LRA motors)
    motors_[WRIST_FLEXION_ZONE] = {.pin = 7, .zone = WRIST_FLEXION_ZONE, .resonant_freq = 175.0};
    motors_[WRIST_ROTATION_ZONE] = {.pin = 8, .zone = WRIST_ROTATION_ZONE, .resonant_freq = 175.0};
    
    // Elbow motor (assume larger LRA)
    motors_[ELBOW_ZONE] = {.pin = 9, .zone = ELBOW_ZONE, .resonant_freq = 150.0};
    
    // Set motor characteristics based on hardware specs
    for (auto& motor : motors_) {
        motor.max_intensity = 0.8;    // Conservative limit
        motor.min_pulse_ms = 80;      // Minimum perceivable pulse
        motor.max_pulse_ms = 400;     // Maximum comfortable pulse
    }
}

inline void LocationSpecificHapticController::triggerFeedback(const HapticCommand& command) {
    if (!system_enabled_ || !validateCommand(command)) {
        return;
    }
    
    // Add to command queue for precise timing
    command_queue_.push(command);
}

inline void LocationSpecificHapticController::triggerLocationSpecificFeedback(
    const std::vector<HapticCommand>& commands) {
    
    // Safety check: Don't overwhelm the user
    if (commands.size() > MAX_SIMULTANEOUS_MOTORS) {
        // Prioritize commands by intensity (keep the strongest feedback)
        std::vector<HapticCommand> sorted_commands = commands;
        std::sort(sorted_commands.begin(), sorted_commands.end(),
                 [](const HapticCommand& a, const HapticCommand& b) {
                     return a.intensity > b.intensity;
                 });
        
        // Only trigger the top N commands
        for (size_t i = 0; i < std::min(static_cast<size_t>(MAX_SIMULTANEOUS_MOTORS), sorted_commands.size()); ++i) {
            triggerFeedback(sorted_commands[i]);
        }
    } else {
        // Trigger all commands
        for (const auto& command : commands) {
            triggerFeedback(command);
        }
    }
}

inline void LocationSpecificHapticController::processCommandQueue() {
    uint64_t current_time = getCurrentTimeMicros();
    
    while (!command_queue_.empty()) {
        HapticCommand& cmd = command_queue_.front();
        
        if (current_time >= cmd.trigger_time_us) {
            // Execute command
            if (cmd.zone < MAX_HAPTIC_ZONES) {
                HapticMotor& motor = motors_[cmd.zone];
                
                // Check inter-pulse timing
                if (current_time - motor.last_trigger_us >= MIN_INTER_PULSE_MS * 1000) {
                    double optimal_intensity = calculateOptimalIntensity(cmd);
                    uint32_t optimal_duration = calculateOptimalDuration(cmd);
                    
                    executeWaveform(motor, static_cast<WaveformPattern>(cmd.pattern_id), 
                                  optimal_intensity, optimal_duration);
                    
                    motor.last_trigger_us = current_time;
                    motor.is_active = true;
                    motor.current_intensity = optimal_intensity;
                }
            }
            
            command_queue_.pop();
        } else {
            break; // Commands are ordered by time
        }
    }
}

inline void LocationSpecificHapticController::executeWaveform(HapticMotor& motor, WaveformPattern pattern, 
                                                             double intensity, uint32_t duration_ms) {
    // Set motor frequency to its resonant frequency for maximum efficiency
    hardwareSetFrequency(motor.pin, motor.resonant_freq);
    
    switch (pattern) {
        case PATTERN_SINGLE_PULSE:
            hardwareSetIntensity(motor.pin, intensity * global_intensity_scale_);
            // Hardware timer will turn off after duration_ms
            break;
            
        case PATTERN_DOUBLE_PULSE:
            // First pulse
            hardwareSetIntensity(motor.pin, intensity * global_intensity_scale_);
            // TODO: Implement timing for second pulse
            break;
            
        case PATTERN_SUCCESS_BURST:
            // Pleasant success pattern - gentle ramp up and down
            // TODO: Implement ramping pattern
            hardwareSetIntensity(motor.pin, intensity * 0.6 * global_intensity_scale_);
            break;
            
        case PATTERN_PRECISION_CLICK:
            // Sharp, precise click for exact positioning feedback
            hardwareSetIntensity(motor.pin, intensity * 0.8 * global_intensity_scale_);
            break;
            
        default:
            hardwareSetIntensity(motor.pin, intensity * global_intensity_scale_);
            break;
    }
}

inline bool LocationSpecificHapticController::validateCommand(const HapticCommand& command) {
    // Validate zone
    if (command.zone >= MAX_HAPTIC_ZONES) return false;
    
    // Validate intensity
    if (command.intensity < 0.0 || command.intensity > 1.0) return false;
    
    // Validate duration
    const HapticMotor& motor = motors_[command.zone];
    if (command.duration_ms < motor.min_pulse_ms || command.duration_ms > motor.max_pulse_ms) {
        return false;
    }
    
    return true;
}

inline double LocationSpecificHapticController::calculateOptimalIntensity(const HapticCommand& command) {
    const HapticMotor& motor = motors_[command.zone];
    
    // Scale intensity based on motor characteristics
    double optimal = command.intensity * motor.max_intensity * global_intensity_scale_;
    
    // Clamp to safe limits
    return std::max(0.0, std::min(optimal, motor.max_intensity));
}

inline uint32_t LocationSpecificHapticController::calculateOptimalDuration(const HapticCommand& command) {
    const HapticMotor& motor = motors_[command.zone];
    
    // Clamp duration to motor limits
    return std::max(motor.min_pulse_ms, std::min(command.duration_ms, motor.max_pulse_ms));
}

inline void LocationSpecificHapticController::stopAll() {
    for (auto& motor : motors_) {
        hardwareSetIntensity(motor.pin, 0.0);
        motor.is_active = false;
        motor.current_intensity = 0.0;
    }
    
    // Clear command queue
    std::queue<HapticCommand> empty;
    command_queue_.swap(empty);
}

inline bool LocationSpecificHapticController::isZoneActive(HapticZone zone) {
    if (zone >= MAX_HAPTIC_ZONES) return false;
    return motors_[zone].is_active;
}

inline void LocationSpecificHapticController::setIntensityScale(double scale) {
    global_intensity_scale_ = std::max(0.0, std::min(scale, 2.0));
}

// Hardware abstraction layer (implement based on your hardware)
inline void LocationSpecificHapticController::hardwareSetIntensity(uint8_t pin, double intensity) {
    // Example for PWM-controlled motors
    uint16_t pwm_value = static_cast<uint16_t>(intensity * 1023);  // 10-bit PWM
    sendPWMSignal(pin, intensity);
    
    // Example for I2C-controlled motors (like DRV2605L)
    // uint8_t i2c_value = static_cast<uint8_t>(intensity * 255);
    // sendI2CCommand(pin, 0x02, i2c_value);  // DRV2605L register 0x02
}

inline void LocationSpecificHapticController::hardwareSetFrequency(uint8_t pin, double frequency) {
    // Implementation depends on your haptic driver hardware
    // For DRV2605L: set library and waveform based on frequency
    // For PWM motors: adjust PWM frequency
}

inline void LocationSpecificHapticController::sendPWMSignal(uint8_t pin, double duty_cycle) {
    // Hardware-specific PWM implementation
    // Example: analogWrite(pin, duty_cycle * 255);
}

inline void LocationSpecificHapticController::sendI2CCommand(uint8_t address, uint8_t command, uint8_t data) {
    // Hardware-specific I2C implementation
    // Example: Wire.beginTransmission(address); Wire.write(command); Wire.write(data); Wire.endTransmission();
}

#endif // LOCATION_HAPTIC_CONTROLLER_H
