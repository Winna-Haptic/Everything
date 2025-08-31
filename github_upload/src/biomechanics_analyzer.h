/*
 * Biomechanics Analyzer for Throwing Form
 * Defines and evaluates "Golden Zones" for optimal throwing mechanics
 * 
 * Core Logic: Location-specific feedback ONLY when biomechanics are correct
 */

#ifndef BIOMECHANICS_ANALYZER_H
#define BIOMECHANICS_ANALYZER_H

#include "architecture.h"
#include <vector>
#include <map>
#include <algorithm>
#include <numeric>

class ThrowingBiomechanicsAnalyzer : public BiomechanicsAnalyzer {
private:
    std::vector<BiomechanicalZone> golden_zones_;
    std::map<ThrowPhase, std::vector<BiomechanicalZone*>> phase_zones_;
    
    // Throw detection state
    ThrowPhase current_phase_;
    uint64_t phase_start_time_;
    std::vector<double> velocity_history_;
    
    // Biomechanical constants (based on sports science research)
    static constexpr double THROW_VELOCITY_THRESHOLD = 50.0;  // deg/s
    static constexpr double RELEASE_DECEL_THRESHOLD = -100.0; // deg/s²
    static constexpr int VELOCITY_HISTORY_SIZE = 10;
    
    // Optimal throwing biomechanics (personalized during calibration)
    struct OptimalBiomechanics {
        // Finger positions at release
        double thumb_flexion_optimal = 45.0;      // degrees
        double index_flexion_optimal = 30.0;     // degrees  
        double middle_flexion_optimal = 35.0;    // degrees
        double ring_flexion_optimal = 40.0;      // degrees
        double pinky_flexion_optimal = 45.0;     // degrees
        
        // Wrist mechanics
        double wrist_extension_optimal = 15.0;    // degrees (slight extension)
        double wrist_ulnar_deviation = 5.0;      // degrees (toward pinky)
        
        // Elbow mechanics
        double elbow_flexion_optimal = 90.0;     // degrees
        double elbow_velocity_peak = 2000.0;     // deg/s
        
        // Timing
        double wind_up_duration = 0.5;           // seconds
        double acceleration_duration = 0.3;      // seconds
        double release_duration = 0.1;           // seconds
        
        // Tolerances (tighter = more precise feedback)
        double finger_tolerance = 8.0;           // degrees
        double wrist_tolerance = 5.0;            // degrees
        double elbow_tolerance = 10.0;           // degrees
        double timing_tolerance = 0.05;          // seconds
    };
    
    OptimalBiomechanics optimal_params_;

public:
    ThrowingBiomechanicsAnalyzer() : current_phase_(PREPARATION), phase_start_time_(0) {
        velocity_history_.reserve(VELOCITY_HISTORY_SIZE);
    }
    
    // BiomechanicsAnalyzer interface
    void setGoldenZones(const std::vector<BiomechanicalZone>& zones) override;
    ThrowPhase detectPhase(const std::vector<JointState>& joints) override;
    bool isInGoldenZone(const JointState& joint, const BiomechanicalZone& zone) override;
    double calculateZoneScore(const JointState& joint, const BiomechanicalZone& zone) override;
    
    // Throwing-specific methods
    void calibrateOptimalForm(const std::vector<ThrowSession>& successful_throws);
    std::vector<HapticCommand> generateLocationSpecificFeedback(const std::vector<JointState>& joints);
    
    // Golden zone management
    void createDefaultGoldenZones();
    void personalizeZones(const std::vector<double>& successful_angles, SensorLocation sensor);
    
    // Phase detection
    bool isThrowStarting(const std::vector<JointState>& joints);
    bool isInReleasePhase(const std::vector<JointState>& joints);
    bool isThrowComplete(const std::vector<JointState>& joints);
    
    // Biomechanical validation
    bool validateFingerPositions(const std::vector<JointState>& joints);
    bool validateWristMechanics(const JointState& wrist_joint);
    bool validateElbowMechanics(const JointState& elbow_joint);
    
    // Analysis utilities
    double calculateFormConsistency(const std::vector<ThrowSession>& sessions);
    std::vector<SensorLocation> identifyWeakZones(const ThrowSession& session);
    
private:
    // Phase detection helpers
    double calculateOverallVelocity(const std::vector<JointState>& joints);
    double calculateAcceleration(const std::vector<double>& velocity_history);
    bool isVelocityIncreasing(const std::vector<double>& history);
    bool isVelocityDecreasing(const std::vector<double>& history);
    
    // Zone evaluation helpers
    bool isAngleInRange(double angle, double optimal, double tolerance);
    double calculateAngleScore(double angle, double optimal, double tolerance);
    double calculateVelocityScore(double velocity, double optimal_min, double optimal_max);
    
    // Calibration helpers
    double calculateOptimalAngle(const std::vector<double>& successful_angles);
    double calculateOptimalTolerance(const std::vector<double>& angles, double optimal);
};

// ===== IMPLEMENTATION =====

inline void ThrowingBiomechanicsAnalyzer::setGoldenZones(const std::vector<BiomechanicalZone>& zones) {
    golden_zones_ = zones;
    
    // Organize zones by phase for efficient lookup
    phase_zones_.clear();
    for (auto& zone : golden_zones_) {
        phase_zones_[zone.phase].push_back(const_cast<BiomechanicalZone*>(&zone));
    }
}

inline ThrowPhase ThrowingBiomechanicsAnalyzer::detectPhase(const std::vector<JointState>& joints) {
    double overall_velocity = calculateOverallVelocity(joints);
    
    // Update velocity history
    velocity_history_.push_back(overall_velocity);
    if (velocity_history_.size() > VELOCITY_HISTORY_SIZE) {
        velocity_history_.erase(velocity_history_.begin());
    }
    
    double acceleration = calculateAcceleration(velocity_history_);
    
    // Phase detection state machine
    switch (current_phase_) {
        case PREPARATION:
            if (overall_velocity > THROW_VELOCITY_THRESHOLD) {
                current_phase_ = WIND_UP;
                phase_start_time_ = getCurrentTimeMicros();
            }
            break;
            
        case WIND_UP:
            if (isVelocityIncreasing(velocity_history_) && acceleration > 0) {
                current_phase_ = ACCELERATION;
                phase_start_time_ = getCurrentTimeMicros();
            }
            break;
            
        case ACCELERATION:
            if (acceleration < RELEASE_DECEL_THRESHOLD) {
                current_phase_ = RELEASE;
                phase_start_time_ = getCurrentTimeMicros();
            }
            break;
            
        case RELEASE:
            if (overall_velocity < THROW_VELOCITY_THRESHOLD * 0.3) {
                current_phase_ = FOLLOW_THROUGH;
                phase_start_time_ = getCurrentTimeMicros();
            }
            break;
            
        case FOLLOW_THROUGH:
            if (overall_velocity < THROW_VELOCITY_THRESHOLD * 0.1) {
                current_phase_ = RECOVERY;
                phase_start_time_ = getCurrentTimeMicros();
            }
            break;
            
        case RECOVERY:
            if (overall_velocity < 10.0) {  // Near stationary
                current_phase_ = PREPARATION;
                phase_start_time_ = getCurrentTimeMicros();
            }
            break;
    }
    
    return current_phase_;
}

inline bool ThrowingBiomechanicsAnalyzer::isInGoldenZone(const JointState& joint, const BiomechanicalZone& zone) {
    if (!joint.is_valid) return false;
    
    // Check if joint angles are within optimal ranges
    bool flexion_ok = isAngleInRange(joint.flexion_angle, 
                                   (zone.min_flexion + zone.max_flexion) / 2.0,
                                   zone.tolerance_degrees);
    
    bool extension_ok = isAngleInRange(joint.extension_angle,
                                     (zone.min_extension + zone.max_extension) / 2.0,
                                     zone.tolerance_degrees);
    
    bool velocity_ok = (joint.angular_velocity >= zone.min_velocity && 
                       joint.angular_velocity <= zone.max_velocity);
    
    return flexion_ok && extension_ok && velocity_ok;
}

inline double ThrowingBiomechanicsAnalyzer::calculateZoneScore(const JointState& joint, const BiomechanicalZone& zone) {
    if (!joint.is_valid) return 0.0;
    
    double flexion_score = calculateAngleScore(joint.flexion_angle,
                                             (zone.min_flexion + zone.max_flexion) / 2.0,
                                             zone.tolerance_degrees);
    
    double extension_score = calculateAngleScore(joint.extension_angle,
                                               (zone.min_extension + zone.max_extension) / 2.0,
                                               zone.tolerance_degrees);
    
    double velocity_score = calculateVelocityScore(joint.angular_velocity,
                                                 zone.min_velocity,
                                                 zone.max_velocity);
    
    // Weighted average (can be tuned)
    return (flexion_score * 0.4 + extension_score * 0.4 + velocity_score * 0.2);
}

inline std::vector<HapticCommand> ThrowingBiomechanicsAnalyzer::generateLocationSpecificFeedback(
    const std::vector<JointState>& joints) {
    
    std::vector<HapticCommand> commands;
    
    // Only generate feedback for current phase
    auto phase_zones = phase_zones_[current_phase_];
    
    for (auto* zone : phase_zones) {
        if (zone->sensor_id >= joints.size()) continue;
        
        const JointState& joint = joints[zone->sensor_id];
        
        // CRITICAL: Only provide haptic feedback when biomechanics are CORRECT
        if (isInGoldenZone(joint, *zone)) {
            double score = calculateZoneScore(joint, *zone);
            
            if (score > zone->confidence_threshold) {
                HapticCommand cmd;
                cmd.zone = zone->haptic_id;
                cmd.intensity = score;  // Intensity proportional to how "perfect" the form is
                cmd.duration_ms = 150;  // Short, pleasant pulse
                cmd.pattern_id = 1;     // Success pattern
                cmd.trigger_time_us = getCurrentTimeMicros();
                cmd.is_active = true;
                
                commands.push_back(cmd);
            }
        }
        // NOTE: No haptic feedback when form is poor - silent learning!
    }
    
    return commands;
}

inline void ThrowingBiomechanicsAnalyzer::createDefaultGoldenZones() {
    golden_zones_.clear();
    
    // Create zones for each finger during release phase
    for (int finger = THUMB_TIP; finger <= PINKY_TIP; ++finger) {
        BiomechanicalZone zone;
        zone.sensor_id = static_cast<SensorLocation>(finger);
        zone.haptic_id = static_cast<HapticZone>(finger);
        zone.phase = RELEASE;
        
        // Set optimal ranges based on finger
        switch (finger) {
            case THUMB_TIP:
                zone.min_flexion = optimal_params_.thumb_flexion_optimal - optimal_params_.finger_tolerance;
                zone.max_flexion = optimal_params_.thumb_flexion_optimal + optimal_params_.finger_tolerance;
                break;
            case INDEX_TIP:
                zone.min_flexion = optimal_params_.index_flexion_optimal - optimal_params_.finger_tolerance;
                zone.max_flexion = optimal_params_.index_flexion_optimal + optimal_params_.finger_tolerance;
                break;
            // ... similar for other fingers
        }
        
        zone.tolerance_degrees = optimal_params_.finger_tolerance;
        zone.confidence_threshold = 0.75;  // 75% confidence required
        
        golden_zones_.push_back(zone);
    }
    
    // Create wrist zone
    BiomechanicalZone wrist_zone;
    wrist_zone.sensor_id = WRIST;
    wrist_zone.haptic_id = WRIST_FLEXION_ZONE;
    wrist_zone.phase = RELEASE;
    wrist_zone.min_extension = optimal_params_.wrist_extension_optimal - optimal_params_.wrist_tolerance;
    wrist_zone.max_extension = optimal_params_.wrist_extension_optimal + optimal_params_.wrist_tolerance;
    wrist_zone.tolerance_degrees = optimal_params_.wrist_tolerance;
    wrist_zone.confidence_threshold = 0.8;
    
    golden_zones_.push_back(wrist_zone);
    
    // Create elbow zone
    BiomechanicalZone elbow_zone;
    elbow_zone.sensor_id = ELBOW;
    elbow_zone.haptic_id = ELBOW_ZONE;
    elbow_zone.phase = ACCELERATION;
    elbow_zone.min_flexion = optimal_params_.elbow_flexion_optimal - optimal_params_.elbow_tolerance;
    elbow_zone.max_flexion = optimal_params_.elbow_flexion_optimal + optimal_params_.elbow_tolerance;
    elbow_zone.min_velocity = optimal_params_.elbow_velocity_peak * 0.8;
    elbow_zone.max_velocity = optimal_params_.elbow_velocity_peak * 1.2;
    elbow_zone.tolerance_degrees = optimal_params_.elbow_tolerance;
    elbow_zone.confidence_threshold = 0.7;
    
    golden_zones_.push_back(elbow_zone);
    
    // Organize by phase
    setGoldenZones(golden_zones_);
}

// Helper function implementations
inline double ThrowingBiomechanicsAnalyzer::calculateOverallVelocity(const std::vector<JointState>& joints) {
    double total_velocity = 0.0;
    int valid_joints = 0;
    
    for (const auto& joint : joints) {
        if (joint.is_valid) {
            total_velocity += std::abs(joint.angular_velocity);
            valid_joints++;
        }
    }
    
    return valid_joints > 0 ? total_velocity / valid_joints : 0.0;
}

inline bool ThrowingBiomechanicsAnalyzer::isAngleInRange(double angle, double optimal, double tolerance) {
    return std::abs(angle - optimal) <= tolerance;
}

inline double ThrowingBiomechanicsAnalyzer::calculateAngleScore(double angle, double optimal, double tolerance) {
    double error = std::abs(angle - optimal);
    if (error > tolerance) return 0.0;
    
    // Linear scoring: perfect = 1.0, at tolerance = 0.0
    return 1.0 - (error / tolerance);
}

#endif // BIOMECHANICS_ANALYZER_H
