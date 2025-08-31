/*
 * Madgwick AHRS Filter Implementation
 * Real-time sensor fusion for haptic glove IMU data
 * 
 * Based on Sebastian Madgwick's gradient descent algorithm
 * Optimized for 100Hz+ operation on low-power MCU
 */

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <array>
#include <cmath>
#include "architecture.h"

class MadgwickFilter : public SensorFusion {
private:
    // Filter parameters
    static constexpr double BETA = 0.1;  // Filter gain (lower = more stable, higher = more responsive)
    static constexpr double SAMPLE_FREQ = 100.0;  // Hz
    static constexpr double GRAVITY = 9.81;  // m/s²
    
    // Per-sensor state (one filter instance per IMU)
    struct FilterState {
        std::array<double, 4> q;  // Quaternion (w, x, y, z)
        double beta;              // Adaptive gain
        uint64_t last_update_us;
        bool is_initialized;
        
        FilterState() : q{1.0, 0.0, 0.0, 0.0}, beta(BETA), 
                       last_update_us(0), is_initialized(false) {}
    };
    
    std::array<FilterState, MAX_SENSORS> filters_;
    bool system_calibrated_;
    
    // Calibration data
    struct CalibrationData {
        std::array<double, 3> accel_bias;
        std::array<double, 3> gyro_bias;
        std::array<double, 3> mag_bias;
        bool is_valid;
        
        CalibrationData() : is_valid(false) {
            accel_bias.fill(0.0);
            gyro_bias.fill(0.0);
            mag_bias.fill(0.0);
        }
    };
    
    std::array<CalibrationData, MAX_SENSORS> calibration_data_;

public:
    MadgwickFilter() : system_calibrated_(false) {}
    
    // SensorFusion interface implementation
    void initialize() override;
    void update(const std::vector<IMUData>& raw_data) override;
    JointState getJointState(SensorLocation location) override;
    std::array<double, 4> getOrientation(SensorLocation location) override;
    bool isCalibrated() override { return system_calibrated_; }
    
    // Madgwick-specific methods
    void updateFilter(SensorLocation sensor, const IMUData& data);
    void calibrateSensor(SensorLocation sensor, const std::vector<IMUData>& static_data);
    
    // Joint angle calculations
    double calculateFingerFlexion(SensorLocation fingertip, SensorLocation hand_back);
    double calculateWristFlexion(SensorLocation wrist, SensorLocation forearm);
    double calculateElbowFlexion(SensorLocation elbow, SensorLocation upper_arm);
    
    // Utility methods
    void setBeta(double beta) { 
        for (auto& filter : filters_) {
            filter.beta = beta;
        }
    }
    
    void resetFilter(SensorLocation sensor);
    double getFilterConfidence(SensorLocation sensor);

private:
    // Core Madgwick algorithm
    void madgwickAHRSupdate(FilterState& state, double gx, double gy, double gz,
                           double ax, double ay, double az, double mx, double my, double mz,
                           double dt);
    
    void madgwickAHRSupdateIMU(FilterState& state, double gx, double gy, double gz,
                              double ax, double ay, double az, double dt);
    
    // Helper functions
    double invSqrt(double x);
    void normalizeQuaternion(std::array<double, 4>& q);
    std::array<double, 3> quaternionToEuler(const std::array<double, 4>& q);
    
    // Calibration helpers
    void calculateBias(const std::vector<IMUData>& data, CalibrationData& cal);
    void applyCalibration(IMUData& data, const CalibrationData& cal);
};

// ===== IMPLEMENTATION =====

inline void MadgwickFilter::initialize() {
    for (auto& filter : filters_) {
        filter.q = {1.0, 0.0, 0.0, 0.0};
        filter.beta = BETA;
        filter.is_initialized = false;
    }
    system_calibrated_ = false;
}

inline void MadgwickFilter::update(const std::vector<IMUData>& raw_data) {
    for (const auto& data : raw_data) {
        if (data.location < MAX_SENSORS) {
            updateFilter(data.location, data);
        }
    }
}

inline void MadgwickFilter::updateFilter(SensorLocation sensor, const IMUData& data) {
    FilterState& state = filters_[sensor];
    
    // Apply calibration if available
    IMUData calibrated_data = data;
    if (calibration_data_[sensor].is_valid) {
        applyCalibration(calibrated_data, calibration_data_[sensor]);
    }
    
    // Calculate time step
    double dt = 1.0 / SAMPLE_FREQ;
    if (state.last_update_us > 0) {
        dt = (data.timestamp_us - state.last_update_us) / 1e6;
    }
    
    // Run Madgwick filter
    if (calibrated_data.mag[0] != 0 || calibrated_data.mag[1] != 0 || calibrated_data.mag[2] != 0) {
        // Full AHRS with magnetometer
        madgwickAHRSupdate(state, 
            calibrated_data.gyro[0], calibrated_data.gyro[1], calibrated_data.gyro[2],
            calibrated_data.accel[0], calibrated_data.accel[1], calibrated_data.accel[2],
            calibrated_data.mag[0], calibrated_data.mag[1], calibrated_data.mag[2],
            dt);
    } else {
        // IMU-only (no magnetometer)
        madgwickAHRSupdateIMU(state,
            calibrated_data.gyro[0], calibrated_data.gyro[1], calibrated_data.gyro[2],
            calibrated_data.accel[0], calibrated_data.accel[1], calibrated_data.accel[2],
            dt);
    }
    
    state.last_update_us = data.timestamp_us;
    state.is_initialized = true;
}

inline JointState MadgwickFilter::getJointState(SensorLocation location) {
    JointState joint;
    
    if (location >= MAX_SENSORS || !filters_[location].is_initialized) {
        joint.is_valid = false;
        return joint;
    }
    
    // Calculate joint angles based on sensor location
    switch (location) {
        case THUMB_TIP:
            joint.flexion_angle = calculateFingerFlexion(THUMB_TIP, BACK_OF_HAND);
            break;
        case INDEX_TIP:
            joint.flexion_angle = calculateFingerFlexion(INDEX_TIP, BACK_OF_HAND);
            break;
        case MIDDLE_TIP:
            joint.flexion_angle = calculateFingerFlexion(MIDDLE_TIP, BACK_OF_HAND);
            break;
        case RING_TIP:
            joint.flexion_angle = calculateFingerFlexion(RING_TIP, BACK_OF_HAND);
            break;
        case PINKY_TIP:
            joint.flexion_angle = calculateFingerFlexion(PINKY_TIP, BACK_OF_HAND);
            break;
        case WRIST:
            joint.flexion_angle = calculateWristFlexion(WRIST, ELBOW);
            break;
        case ELBOW:
            joint.flexion_angle = calculateElbowFlexion(ELBOW, WRIST);
            break;
        default:
            joint.is_valid = false;
            return joint;
    }
    
    // Calculate angular velocity from quaternion derivative
    // TODO: Implement proper angular velocity calculation
    joint.angular_velocity = 0.0;  // Placeholder
    joint.is_valid = true;
    
    return joint;
}

inline std::array<double, 4> MadgwickFilter::getOrientation(SensorLocation location) {
    if (location < MAX_SENSORS && filters_[location].is_initialized) {
        return filters_[location].q;
    }
    return {1.0, 0.0, 0.0, 0.0};  // Identity quaternion
}

inline double MadgwickFilter::calculateFingerFlexion(SensorLocation fingertip, SensorLocation hand_back) {
    // Calculate relative orientation between fingertip and back of hand
    auto q_finger = getOrientation(fingertip);
    auto q_hand = getOrientation(hand_back);
    
    // Compute relative quaternion (finger relative to hand)
    // q_rel = q_finger * conjugate(q_hand)
    std::array<double, 4> q_rel = {
        q_finger[0]*q_hand[0] + q_finger[1]*q_hand[1] + q_finger[2]*q_hand[2] + q_finger[3]*q_hand[3],
        q_finger[1]*q_hand[0] - q_finger[0]*q_hand[1] - q_finger[3]*q_hand[2] + q_finger[2]*q_hand[3],
        q_finger[2]*q_hand[0] + q_finger[3]*q_hand[1] - q_finger[0]*q_hand[2] - q_finger[1]*q_hand[3],
        q_finger[3]*q_hand[0] - q_finger[2]*q_hand[1] + q_finger[1]*q_hand[2] - q_finger[0]*q_hand[3]
    };
    
    // Convert to Euler angle (flexion is typically around Y-axis)
    return quaternionToEuler(q_rel)[1] * 180.0 / M_PI;
}

inline double MadgwickFilter::calculateWristFlexion(SensorLocation wrist, SensorLocation forearm) {
    // Similar calculation for wrist flexion
    auto q_wrist = getOrientation(wrist);
    auto q_forearm = getOrientation(forearm);
    
    // Calculate relative flexion angle
    // Implementation similar to finger flexion
    return 0.0;  // Placeholder
}

inline double MadgwickFilter::calculateElbowFlexion(SensorLocation elbow, SensorLocation upper_arm) {
    // Calculate elbow flexion angle
    return 0.0;  // Placeholder
}

// Fast inverse square root (Quake algorithm)
inline double MadgwickFilter::invSqrt(double x) {
    return 1.0 / std::sqrt(x);  // Modern compilers optimize this well
}

inline std::array<double, 3> MadgwickFilter::quaternionToEuler(const std::array<double, 4>& q) {
    std::array<double, 3> euler;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (std::abs(sinp) >= 1)
        euler[1] = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        euler[1] = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

#endif // MADGWICK_FILTER_H
