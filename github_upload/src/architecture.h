/*
 * Modular Haptic Glove Architecture
 * Location-specific feedback for biomechanically optimal throwing form
 * 
 * Core Philosophy: "Bicycle not Airplane"
 * - Fingers + wrist accuracy first
 * - Single haptic motor per zone (if optimal)
 * - Rock-solid sense → decide → precise vibration loop
 */

#ifndef ARCHITECTURE_H
#define ARCHITECTURE_H

#include <vector>
#include <array>
#include <chrono>
#include <functional>

// ===== HARDWARE CONFIGURATION =====

// IMU Sensor Locations (100Hz minimum)
enum SensorLocation {
    THUMB_TIP = 0,
    INDEX_TIP = 1,
    MIDDLE_TIP = 2,
    RING_TIP = 3,
    PINKY_TIP = 4,
    BACK_OF_HAND = 5,
    WRIST = 6,
    ELBOW = 7,
    MAX_SENSORS = 8
};

// Haptic Motor Locations (1:1 mapping with biomechanical zones)
enum HapticZone {
    THUMB_ZONE = 0,
    INDEX_ZONE = 1,
    MIDDLE_ZONE = 2,
    RING_ZONE = 3,
    PINKY_ZONE = 4,
    WRIST_FLEXION_ZONE = 5,
    WRIST_ROTATION_ZONE = 6,
    ELBOW_ZONE = 7,
    MAX_HAPTIC_ZONES = 8
};

// Throw Phases (biomechanical sequence)
enum ThrowPhase {
    PREPARATION,    // Hand positioning, grip setup
    WIND_UP,        // Arm cocking, energy loading
    ACCELERATION,   // Forward motion, power generation  
    RELEASE,        // Ball release, finger extension
    FOLLOW_THROUGH, // Deceleration, wrist snap
    RECOVERY        // Return to neutral
};

// ===== DATA STRUCTURES =====

// High-frequency IMU data (100Hz+)
struct IMUData {
    std::array<double, 3> accel;     // m/s² in body frame
    std::array<double, 3> gyro;      // rad/s in body frame
    std::array<double, 3> mag;       // μT (if available)
    std::array<double, 4> quaternion; // Orientation (w,x,y,z)
    uint64_t timestamp_us;           // Microsecond precision
    SensorLocation location;
    
    IMUData() : timestamp_us(0), location(MAX_SENSORS) {
        accel.fill(0.0);
        gyro.fill(0.0);
        mag.fill(0.0);
        quaternion = {1.0, 0.0, 0.0, 0.0}; // Identity quaternion
    }
};

// Biomechanical joint state
struct JointState {
    double flexion_angle;      // degrees (-180 to +180)
    double extension_angle;    // degrees (0 to max ROM)
    double rotation_angle;     // degrees (pronation/supination)
    double angular_velocity;   // deg/s
    double angular_accel;      // deg/s²
    bool is_valid;            // Data quality flag
    
    JointState() : flexion_angle(0), extension_angle(0), rotation_angle(0),
                   angular_velocity(0), angular_accel(0), is_valid(false) {}
};

// Biomechanical "Golden Zone" definition
struct BiomechanicalZone {
    SensorLocation sensor_id;
    HapticZone haptic_id;
    ThrowPhase phase;
    
    // Optimal ranges (personalized during calibration)
    double min_flexion, max_flexion;
    double min_extension, max_extension; 
    double min_rotation, max_rotation;
    double min_velocity, max_velocity;
    
    // Zone tolerance (tighter = more precise feedback)
    double tolerance_degrees;
    double tolerance_velocity;
    
    // Confidence weighting (0.0 to 1.0)
    double confidence_threshold;
    
    BiomechanicalZone() : sensor_id(MAX_SENSORS), haptic_id(MAX_HAPTIC_ZONES),
                         phase(PREPARATION), min_flexion(0), max_flexion(0),
                         min_extension(0), max_extension(0), min_rotation(0),
                         max_rotation(0), min_velocity(0), max_velocity(0),
                         tolerance_degrees(5.0), tolerance_velocity(10.0),
                         confidence_threshold(0.8) {}
};

// Real-time haptic command
struct HapticCommand {
    HapticZone zone;
    double intensity;        // 0.0 to 1.0
    uint32_t duration_ms;    // Pulse length
    uint32_t pattern_id;     // Waveform type
    uint64_t trigger_time_us; // When to fire
    bool is_active;
    
    HapticCommand() : zone(MAX_HAPTIC_ZONES), intensity(0.0), duration_ms(0),
                     pattern_id(0), trigger_time_us(0), is_active(false) {}
};

// Session data for analysis
struct ThrowSession {
    uint64_t session_id;
    std::vector<IMUData> sensor_data;
    std::vector<HapticCommand> haptic_log;
    std::vector<BiomechanicalZone> active_zones;
    ThrowPhase current_phase;
    uint32_t zone_hits;      // Successful zone entries
    uint32_t total_throws;
    double session_score;    // Overall performance
    
    ThrowSession() : session_id(0), current_phase(PREPARATION),
                    zone_hits(0), total_throws(0), session_score(0.0) {}
};

// ===== MODULAR SYSTEM INTERFACES =====

// Sensor fusion interface (pluggable filters)
class SensorFusion {
public:
    virtual ~SensorFusion() = default;
    virtual void initialize() = 0;
    virtual void update(const std::vector<IMUData>& raw_data) = 0;
    virtual JointState getJointState(SensorLocation location) = 0;
    virtual std::array<double, 4> getOrientation(SensorLocation location) = 0;
    virtual bool isCalibrated() = 0;
};

// Biomechanical analysis interface
class BiomechanicsAnalyzer {
public:
    virtual ~BiomechanicsAnalyzer() = default;
    virtual void setGoldenZones(const std::vector<BiomechanicalZone>& zones) = 0;
    virtual ThrowPhase detectPhase(const std::vector<JointState>& joints) = 0;
    virtual bool isInGoldenZone(const JointState& joint, const BiomechanicalZone& zone) = 0;
    virtual double calculateZoneScore(const JointState& joint, const BiomechanicalZone& zone) = 0;
};

// Haptic control interface
class HapticController {
public:
    virtual ~HapticController() = default;
    virtual void initialize() = 0;
    virtual void triggerFeedback(const HapticCommand& command) = 0;
    virtual void stopAll() = 0;
    virtual bool isZoneActive(HapticZone zone) = 0;
    virtual void setIntensityScale(double scale) = 0;
};

// Communication interface (BLE, USB, etc.)
class CommunicationInterface {
public:
    virtual ~CommunicationInterface() = default;
    virtual void initialize() = 0;
    virtual void sendSensorData(const std::vector<IMUData>& data) = 0;
    virtual void sendHapticLog(const std::vector<HapticCommand>& commands) = 0;
    virtual void sendSessionSummary(const ThrowSession& session) = 0;
    virtual bool receiveCalibrationData(std::vector<BiomechanicalZone>& zones) = 0;
};

// ===== CORE SYSTEM CLASS =====

class HapticGloveSystem {
private:
    // Modular components (dependency injection)
    std::unique_ptr<SensorFusion> sensor_fusion_;
    std::unique_ptr<BiomechanicsAnalyzer> biomechanics_;
    std::unique_ptr<HapticController> haptic_controller_;
    std::unique_ptr<CommunicationInterface> communication_;
    
    // System state
    std::vector<BiomechanicalZone> golden_zones_;
    ThrowSession current_session_;
    bool is_calibrated_;
    bool is_active_;
    
    // Real-time buffers
    std::vector<IMUData> sensor_buffer_;
    std::vector<JointState> joint_buffer_;
    std::vector<HapticCommand> haptic_buffer_;
    
    // Timing
    uint64_t last_update_us_;
    const uint32_t UPDATE_RATE_HZ = 100;
    const uint64_t UPDATE_PERIOD_US = 1000000 / UPDATE_RATE_HZ;

public:
    HapticGloveSystem() : is_calibrated_(false), is_active_(false), last_update_us_(0) {}
    
    // System lifecycle
    void initialize(
        std::unique_ptr<SensorFusion> fusion,
        std::unique_ptr<BiomechanicsAnalyzer> analyzer,
        std::unique_ptr<HapticController> haptic,
        std::unique_ptr<CommunicationInterface> comm
    );
    
    void startSession();
    void stopSession();
    void calibrate();
    
    // Real-time processing loop (100Hz)
    void update();
    
    // Core feedback logic
    void processSensorData();
    void analyzebiomechanics();
    void triggerLocationSpecificFeedback();
    
    // Configuration
    void setGoldenZones(const std::vector<BiomechanicalZone>& zones);
    void updateZoneTolerance(HapticZone zone, double tolerance);
    
    // Status
    bool isCalibrated() const { return is_calibrated_; }
    bool isActive() const { return is_active_; }
    uint32_t getZoneHits() const { return current_session_.zone_hits; }
    double getSessionScore() const { return current_session_.session_score; }
};

// ===== UTILITY FUNCTIONS =====

// Timing utilities
uint64_t getCurrentTimeMicros();
bool isTimeForUpdate(uint64_t last_time, uint64_t period_us);

// Math utilities
double quaternionToEuler(const std::array<double, 4>& quat, int axis);
double calculateAngularVelocity(double current_angle, double previous_angle, double dt);
bool isWithinTolerance(double value, double target, double tolerance);

// Calibration utilities
std::vector<BiomechanicalZone> generatePersonalizedZones(const std::vector<ThrowSession>& calibration_data);
double calculateOptimalRange(const std::vector<double>& values, double percentile);

#endif // ARCHITECTURE_H
