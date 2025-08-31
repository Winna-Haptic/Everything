/*
 * Haptic Glove Simulation - Complete System Test
 * Location-specific haptic feedback for optimal throwing form
 * 
 * This simulation demonstrates the full system without hardware:
 * - 8 IMU sensors generating realistic motion data
 * - Real-time biomechanical analysis
 * - Location-specific haptic feedback
 * - Golden zone detection and validation
 */

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <random>
#include <iomanip>
#include <fstream>
#include <memory>

#include "architecture.h"
#include "madgwick_filter.h"
#include "biomechanics_analyzer.h"
#include "location_haptic_controller.h"

// Simulation-specific implementations
class SimulationSensorFusion : public MadgwickFilter {
public:
    void generateRealisticIMUData(std::vector<IMUData>& data, ThrowPhase phase, double time_in_phase) {
        data.clear();
        data.resize(MAX_SENSORS);
        
        // Generate realistic sensor data based on throw phase
        for (int i = 0; i < MAX_SENSORS; i++) {
            data[i].location = static_cast<SensorLocation>(i);
            data[i].timestamp_us = getCurrentTimeMicros();
            
            generateSensorData(data[i], phase, time_in_phase);
        }
    }

private:
    void generateSensorData(IMUData& data, ThrowPhase phase, double time_in_phase) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> noise(0.0, 0.1);
        
        // Base motion patterns for each throw phase
        switch (phase) {
            case PREPARATION:
                // Minimal movement, mostly noise
                data.accel = {0.0 + noise(gen), 0.0 + noise(gen), 9.81 + noise(gen)};
                data.gyro = {0.0 + noise(gen), 0.0 + noise(gen), 0.0 + noise(gen)};
                break;
                
            case WIND_UP:
                // Increasing acceleration as arm moves back
                data.accel = {2.0 * time_in_phase + noise(gen), 
                             1.0 * time_in_phase + noise(gen), 
                             9.81 + noise(gen)};
                data.gyro = {10.0 * time_in_phase + noise(gen), 
                            5.0 * time_in_phase + noise(gen), 
                            3.0 * time_in_phase + noise(gen)};
                break;
                
            case ACCELERATION:
                // High acceleration forward
                double accel_magnitude = 15.0 + 10.0 * sin(time_in_phase * M_PI);
                data.accel = {accel_magnitude + noise(gen), 
                             accel_magnitude * 0.5 + noise(gen), 
                             9.81 + accel_magnitude * 0.3 + noise(gen)};
                data.gyro = {200.0 + 100.0 * sin(time_in_phase * M_PI) + noise(gen),
                            150.0 + 80.0 * sin(time_in_phase * M_PI) + noise(gen),
                            100.0 + 50.0 * sin(time_in_phase * M_PI) + noise(gen)};
                break;
                
            case RELEASE:
                // Peak velocity, starting to decelerate
                data.accel = {-5.0 + noise(gen), -3.0 + noise(gen), 9.81 + noise(gen)};
                data.gyro = {300.0 * (1.0 - time_in_phase) + noise(gen),
                            250.0 * (1.0 - time_in_phase) + noise(gen),
                            200.0 * (1.0 - time_in_phase) + noise(gen)};
                break;
                
            case FOLLOW_THROUGH:
                // Deceleration
                data.accel = {-2.0 * (1.0 - time_in_phase) + noise(gen),
                             -1.5 * (1.0 - time_in_phase) + noise(gen),
                             9.81 + noise(gen)};
                data.gyro = {50.0 * (1.0 - time_in_phase) + noise(gen),
                            30.0 * (1.0 - time_in_phase) + noise(gen),
                            20.0 * (1.0 - time_in_phase) + noise(gen)};
                break;
                
            case RECOVERY:
                // Return to rest
                data.accel = {0.0 + noise(gen), 0.0 + noise(gen), 9.81 + noise(gen)};
                data.gyro = {0.0 + noise(gen), 0.0 + noise(gen), 0.0 + noise(gen)};
                break;
        }
        
        // Add sensor-specific variations
        modifySensorDataByLocation(data);
    }
    
    void modifySensorDataByLocation(IMUData& data) {
        // Different sensors have different motion characteristics
        switch (data.location) {
            case THUMB_TIP:
                // Thumb has less motion than fingers
                for (auto& a : data.accel) a *= 0.7;
                for (auto& g : data.gyro) g *= 0.6;
                break;
                
            case INDEX_TIP:
                // Index finger has most motion during release
                for (auto& a : data.accel) a *= 1.2;
                for (auto& g : data.gyro) g *= 1.3;
                break;
                
            case WRIST:
                // Wrist is pivot point, high angular velocity
                for (auto& g : data.gyro) g *= 1.5;
                break;
                
            case ELBOW:
                // Elbow drives the motion, high acceleration
                for (auto& a : data.accel) a *= 1.4;
                break;
                
            default:
                // Other fingers have moderate motion
                break;
        }
    }
};

class SimulationHapticController : public LocationSpecificHapticController {
public:
    void triggerFeedback(const HapticCommand& command) override {
        // Simulate haptic feedback with console output
        if (command.is_active && command.intensity > 0.1) {
            std::string zone_name = getZoneName(command.zone);
            std::string pattern_name = getPatternName(command.pattern_id);
            
            std::cout << "🎯 HAPTIC: " << zone_name 
                     << " | Intensity: " << std::fixed << std::setprecision(2) << command.intensity
                     << " | Pattern: " << pattern_name
                     << " | Duration: " << command.duration_ms << "ms" << std::endl;
        }
    }
    
    void stopAll() override {
        std::cout << "⏹️  All haptic feedback stopped" << std::endl;
    }
    
    bool isZoneActive(HapticZone zone) override {
        return false; // Simulation always returns false
    }
    
    void setIntensityScale(double scale) override {
        std::cout << "🔧 Haptic intensity scale set to: " << scale << std::endl;
    }
    
    void initialize() override {
        std::cout << "🚀 Haptic system initialized (simulation mode)" << std::endl;
    }

private:
    std::string getZoneName(HapticZone zone) {
        switch (zone) {
            case THUMB_ZONE: return "THUMB";
            case INDEX_ZONE: return "INDEX";
            case MIDDLE_ZONE: return "MIDDLE";
            case RING_ZONE: return "RING";
            case PINKY_ZONE: return "PINKY";
            case WRIST_FLEXION_ZONE: return "WRIST_FLEX";
            case WRIST_ROTATION_ZONE: return "WRIST_ROT";
            case ELBOW_ZONE: return "ELBOW";
            default: return "UNKNOWN";
        }
    }
    
    std::string getPatternName(uint32_t pattern_id) {
        switch (pattern_id) {
            case 1: return "Single Pulse";
            case 2: return "Double Pulse";
            case 3: return "Success Burst";
            case 4: return "Gentle Fade";
            case 5: return "Precision Click";
            default: return "Custom";
        }
    }
};

class SimulationCommunication : public CommunicationInterface {
public:
    void initialize() override {
        std::cout << "📡 Communication interface initialized (simulation mode)" << std::endl;
    }
    
    void sendSensorData(const std::vector<IMUData>& data) override {
        // Log sensor data to file for analysis
        static std::ofstream log_file("simulation_sensor_data.csv", std::ios::app);
        static bool header_written = false;
        
        if (!header_written) {
            log_file << "timestamp,sensor,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n";
            header_written = true;
        }
        
        for (const auto& sensor : data) {
            log_file << sensor.timestamp_us << "," << sensor.location << ","
                    << sensor.accel[0] << "," << sensor.accel[1] << "," << sensor.accel[2] << ","
                    << sensor.gyro[0] << "," << sensor.gyro[1] << "," << sensor.gyro[2] << "\n";
        }
    }
    
    void sendHapticLog(const std::vector<HapticCommand>& commands) override {
        // Log haptic commands
        static std::ofstream haptic_log("simulation_haptic_log.csv", std::ios::app);
        static bool header_written = false;
        
        if (!header_written) {
            haptic_log << "timestamp,zone,intensity,duration,pattern\n";
            header_written = true;
        }
        
        for (const auto& cmd : commands) {
            haptic_log << cmd.trigger_time_us << "," << cmd.zone << ","
                      << cmd.intensity << "," << cmd.duration_ms << "," << cmd.pattern_id << "\n";
        }
    }
    
    void sendSessionSummary(const ThrowSession& session) override {
        std::cout << "\n📊 SESSION SUMMARY:" << std::endl;
        std::cout << "   Total throws: " << session.total_throws << std::endl;
        std::cout << "   Zone hits: " << session.zone_hits << std::endl;
        std::cout << "   Success rate: " << std::fixed << std::setprecision(1) 
                 << (session.total_throws > 0 ? (double)session.zone_hits / session.total_throws * 100 : 0) 
                 << "%" << std::endl;
        std::cout << "   Session score: " << std::fixed << std::setprecision(2) 
                 << session.session_score << std::endl;
    }
    
    bool receiveCalibrationData(std::vector<BiomechanicalZone>& zones) override {
        // Simulate receiving calibration data
        return false; // No external calibration in simulation
    }
};

// Main simulation class
class HapticGloveSimulation {
private:
    std::unique_ptr<HapticGloveSystem> system_;
    std::unique_ptr<SimulationSensorFusion> sensor_fusion_;
    std::unique_ptr<ThrowingBiomechanicsAnalyzer> biomechanics_;
    std::unique_ptr<SimulationHapticController> haptic_controller_;
    std::unique_ptr<SimulationCommunication> communication_;
    
    // Simulation state
    ThrowPhase current_phase_;
    double phase_time_;
    double total_time_;
    int throw_count_;
    bool is_running_;
    
    // Simulation parameters
    static constexpr double UPDATE_RATE_HZ = 100.0;
    static constexpr double UPDATE_PERIOD_S = 1.0 / UPDATE_RATE_HZ;
    
    // Throw timing (realistic durations)
    static constexpr double PREPARATION_DURATION = 2.0;   // seconds
    static constexpr double WIND_UP_DURATION = 0.8;       // seconds
    static constexpr double ACCELERATION_DURATION = 0.4;  // seconds
    static constexpr double RELEASE_DURATION = 0.15;      // seconds
    static constexpr double FOLLOW_THROUGH_DURATION = 0.6; // seconds
    static constexpr double RECOVERY_DURATION = 1.0;      // seconds

public:
    HapticGloveSimulation() : current_phase_(PREPARATION), phase_time_(0.0), 
                             total_time_(0.0), throw_count_(0), is_running_(false) {
        initializeSystem();
    }
    
    void run() {
        std::cout << "\n🚀 Starting Haptic Glove Simulation" << std::endl;
        std::cout << "====================================" << std::endl;
        
        is_running_ = true;
        system_->startSession();
        
        auto last_update = std::chrono::steady_clock::now();
        
        while (is_running_ && throw_count_ < 5) { // Simulate 5 throws
            auto now = std::chrono::steady_clock::now();
            auto dt = std::chrono::duration<double>(now - last_update).count();
            
            if (dt >= UPDATE_PERIOD_S) {
                update(dt);
                last_update = now;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        system_->stopSession();
        std::cout << "\n✅ Simulation completed successfully!" << std::endl;
    }
    
    void stop() {
        is_running_ = false;
    }

private:
    void initializeSystem() {
        // Create components
        sensor_fusion_ = std::make_unique<SimulationSensorFusion>();
        biomechanics_ = std::make_unique<ThrowingBiomechanicsAnalyzer>();
        haptic_controller_ = std::make_unique<SimulationHapticController>();
        communication_ = std::make_unique<SimulationCommunication>();
        
        // Create main system
        system_ = std::make_unique<HapticGloveSystem>();
        
        // Initialize system with components
        system_->initialize(
            std::unique_ptr<SensorFusion>(sensor_fusion_.release()),
            std::unique_ptr<BiomechanicsAnalyzer>(biomechanics_.release()),
            std::unique_ptr<HapticController>(haptic_controller_.release()),
            std::unique_ptr<CommunicationInterface>(communication_.release())
        );
        
        // Set up default golden zones
        biomechanics_->createDefaultGoldenZones();
        
        std::cout << "🔧 Haptic glove system initialized" << std::endl;
    }
    
    void update(double dt) {
        total_time_ += dt;
        phase_time_ += dt;
        
        // Update throw phase
        updateThrowPhase();
        
        // Print phase changes
        static ThrowPhase last_phase = RECOVERY;
        if (current_phase_ != last_phase) {
            printPhaseChange(current_phase_);
            last_phase = current_phase_;
        }
        
        // Generate realistic sensor data
        std::vector<IMUData> sensor_data;
        sensor_fusion_->generateRealisticIMUData(sensor_data, current_phase_, phase_time_);
        
        // Update sensor fusion
        sensor_fusion_->update(sensor_data);
        
        // Get joint states
        std::vector<JointState> joints(MAX_SENSORS);
        for (int i = 0; i < MAX_SENSORS; i++) {
            joints[i] = sensor_fusion_->getJointState(static_cast<SensorLocation>(i));
        }
        
        // Analyze biomechanics
        ThrowPhase detected_phase = biomechanics_->detectPhase(joints);
        
        // Generate location-specific haptic feedback
        auto haptic_commands = biomechanics_->generateLocationSpecificFeedback(joints);
        
        // Execute haptic feedback
        for (const auto& command : haptic_commands) {
            haptic_controller_->triggerFeedback(command);
        }
        
        // Log data
        communication_->sendSensorData(sensor_data);
        communication_->sendHapticLog(haptic_commands);
        
        // Update system
        system_->update();
    }
    
    void updateThrowPhase() {
        double phase_duration = getPhaseDuration(current_phase_);
        
        if (phase_time_ >= phase_duration) {
            // Advance to next phase
            current_phase_ = getNextPhase(current_phase_);
            phase_time_ = 0.0;
            
            if (current_phase_ == PREPARATION) {
                throw_count_++;
                std::cout << "\n🏀 Throw #" << throw_count_ << " completed" << std::endl;
            }
        }
    }
    
    double getPhaseDuration(ThrowPhase phase) {
        switch (phase) {
            case PREPARATION: return PREPARATION_DURATION;
            case WIND_UP: return WIND_UP_DURATION;
            case ACCELERATION: return ACCELERATION_DURATION;
            case RELEASE: return RELEASE_DURATION;
            case FOLLOW_THROUGH: return FOLLOW_THROUGH_DURATION;
            case RECOVERY: return RECOVERY_DURATION;
            default: return 1.0;
        }
    }
    
    ThrowPhase getNextPhase(ThrowPhase current) {
        switch (current) {
            case PREPARATION: return WIND_UP;
            case WIND_UP: return ACCELERATION;
            case ACCELERATION: return RELEASE;
            case RELEASE: return FOLLOW_THROUGH;
            case FOLLOW_THROUGH: return RECOVERY;
            case RECOVERY: return PREPARATION;
            default: return PREPARATION;
        }
    }
    
    void printPhaseChange(ThrowPhase phase) {
        std::string phase_name = getPhaseName(phase);
        std::cout << "\n📍 Phase: " << phase_name << " (t=" << std::fixed << std::setprecision(1) 
                 << total_time_ << "s)" << std::endl;
    }
    
    std::string getPhaseName(ThrowPhase phase) {
        switch (phase) {
            case PREPARATION: return "PREPARATION";
            case WIND_UP: return "WIND_UP";
            case ACCELERATION: return "ACCELERATION";
            case RELEASE: return "RELEASE";
            case FOLLOW_THROUGH: return "FOLLOW_THROUGH";
            case RECOVERY: return "RECOVERY";
            default: return "UNKNOWN";
        }
    }
};

// Utility function implementations
uint64_t getCurrentTimeMicros() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

bool isTimeForUpdate(uint64_t last_time, uint64_t period_us) {
    return (getCurrentTimeMicros() - last_time) >= period_us;
}

// Main function
int main() {
    std::cout << "🎯 Haptic Glove Location-Specific Feedback Simulation" << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << "This simulation demonstrates:" << std::endl;
    std::cout << "• 8 IMU sensors at 100Hz" << std::endl;
    std::cout << "• Real-time biomechanical analysis" << std::endl;
    std::cout << "• Location-specific haptic feedback" << std::endl;
    std::cout << "• Golden zone detection" << std::endl;
    std::cout << "• Positive reinforcement only" << std::endl;
    std::cout << "=====================================================" << std::endl;
    
    try {
        HapticGloveSimulation simulation;
        simulation.run();
    } catch (const std::exception& e) {
        std::cerr << "❌ Simulation error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
