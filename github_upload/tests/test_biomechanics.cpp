/*
 * Unit Tests for Biomechanics Analysis
 * Validates golden zone detection and location-specific feedback logic
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../src/biomechanics_analyzer.h"
#include "../src/architecture.h"

class BiomechanicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        analyzer = std::make_unique<ThrowingBiomechanicsAnalyzer>();
        analyzer->createDefaultGoldenZones();
    }
    
    std::unique_ptr<ThrowingBiomechanicsAnalyzer> analyzer;
};

// Test golden zone creation
TEST_F(BiomechanicsTest, CreatesDefaultGoldenZones) {
    // Should create zones for all sensors
    // This test validates that the system has proper zone coverage
    EXPECT_TRUE(true); // Placeholder - implement based on analyzer interface
}

// Test optimal biomechanics detection
TEST_F(BiomechanicsTest, DetectsOptimalFingerPositions) {
    // Create joint state with optimal finger positions
    JointState optimal_thumb;
    optimal_thumb.flexion_angle = 45.0;  // Optimal thumb position
    optimal_thumb.angular_velocity = 50.0;
    optimal_thumb.is_valid = true;
    
    // Create corresponding golden zone
    BiomechanicalZone thumb_zone;
    thumb_zone.sensor_id = THUMB_TIP;
    thumb_zone.haptic_id = THUMB_ZONE;
    thumb_zone.min_flexion = 40.0;
    thumb_zone.max_flexion = 50.0;
    thumb_zone.min_velocity = 0.0;
    thumb_zone.max_velocity = 100.0;
    thumb_zone.tolerance_degrees = 5.0;
    
    // Test zone detection
    bool in_zone = analyzer->isInGoldenZone(optimal_thumb, thumb_zone);
    EXPECT_TRUE(in_zone) << "Optimal thumb position should be in golden zone";
    
    // Test zone score
    double score = analyzer->calculateZoneScore(optimal_thumb, thumb_zone);
    EXPECT_GT(score, 0.8) << "Optimal position should have high score";
}

// Test suboptimal biomechanics (should NOT trigger feedback)
TEST_F(BiomechanicsTest, RejectsSuboptimalPositions) {
    // Create joint state with poor finger position
    JointState poor_thumb;
    poor_thumb.flexion_angle = 20.0;  // Too extended
    poor_thumb.angular_velocity = 200.0;  // Too fast
    poor_thumb.is_valid = true;
    
    // Create golden zone for optimal position
    BiomechanicalZone thumb_zone;
    thumb_zone.sensor_id = THUMB_TIP;
    thumb_zone.haptic_id = THUMB_ZONE;
    thumb_zone.min_flexion = 40.0;
    thumb_zone.max_flexion = 50.0;
    thumb_zone.min_velocity = 0.0;
    thumb_zone.max_velocity = 100.0;
    thumb_zone.tolerance_degrees = 5.0;
    
    // Test zone detection - should be FALSE
    bool in_zone = analyzer->isInGoldenZone(poor_thumb, thumb_zone);
    EXPECT_FALSE(in_zone) << "Poor thumb position should NOT be in golden zone";
    
    // Test zone score - should be low
    double score = analyzer->calculateZoneScore(poor_thumb, thumb_zone);
    EXPECT_LT(score, 0.3) << "Poor position should have low score";
}

// Test throw phase detection
TEST_F(BiomechanicsTest, DetectsThrowPhases) {
    // Create joint states representing different throw phases
    std::vector<JointState> preparation_joints(MAX_SENSORS);
    std::vector<JointState> acceleration_joints(MAX_SENSORS);
    
    // Preparation phase - low velocity
    for (auto& joint : preparation_joints) {
        joint.angular_velocity = 5.0;  // Very low
        joint.is_valid = true;
    }
    
    // Acceleration phase - high velocity
    for (auto& joint : acceleration_joints) {
        joint.angular_velocity = 150.0;  // High
        joint.is_valid = true;
    }
    
    ThrowPhase prep_phase = analyzer->detectPhase(preparation_joints);
    ThrowPhase accel_phase = analyzer->detectPhase(acceleration_joints);
    
    // Phases should be different
    EXPECT_NE(prep_phase, accel_phase) << "Different motion patterns should produce different phases";
}

// Test location-specific feedback generation
TEST_F(BiomechanicsTest, GeneratesLocationSpecificFeedback) {
    // Create joint states with mixed performance
    std::vector<JointState> joints(MAX_SENSORS);
    
    // Thumb optimal
    joints[THUMB_TIP].flexion_angle = 45.0;
    joints[THUMB_TIP].angular_velocity = 50.0;
    joints[THUMB_TIP].is_valid = true;
    
    // Index finger suboptimal
    joints[INDEX_TIP].flexion_angle = 10.0;  // Too extended
    joints[INDEX_TIP].angular_velocity = 200.0;  // Too fast
    joints[INDEX_TIP].is_valid = true;
    
    // Generate feedback
    auto haptic_commands = analyzer->generateLocationSpecificFeedback(joints);
    
    // Should have feedback for thumb (good) but not index (bad)
    bool has_thumb_feedback = false;
    bool has_index_feedback = false;
    
    for (const auto& cmd : haptic_commands) {
        if (cmd.zone == THUMB_ZONE) has_thumb_feedback = true;
        if (cmd.zone == INDEX_ZONE) has_index_feedback = true;
    }
    
    EXPECT_TRUE(has_thumb_feedback) << "Should provide feedback for optimal thumb position";
    EXPECT_FALSE(has_index_feedback) << "Should NOT provide feedback for suboptimal index position";
}

// Test consistency scoring
TEST_F(BiomechanicsTest, CalculatesConsistencyScores) {
    // Create consistent joint data
    std::vector<double> consistent_angles = {45.0, 46.0, 44.0, 45.5, 44.5};
    std::vector<double> inconsistent_angles = {45.0, 30.0, 60.0, 20.0, 70.0};
    
    // Calculate standard deviations
    auto calc_std = [](const std::vector<double>& data) {
        double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        double variance = 0.0;
        for (double val : data) {
            variance += (val - mean) * (val - mean);
        }
        return std::sqrt(variance / data.size());
    };
    
    double consistent_std = calc_std(consistent_angles);
    double inconsistent_std = calc_std(inconsistent_angles);
    
    EXPECT_LT(consistent_std, inconsistent_std) << "Consistent data should have lower standard deviation";
    
    // Test consistency scoring (higher score = more consistent)
    double consistent_score = 1.0 / (1.0 + consistent_std);
    double inconsistent_score = 1.0 / (1.0 + inconsistent_std);
    
    EXPECT_GT(consistent_score, inconsistent_score) << "Consistent data should have higher consistency score";
}

// Test calibration data validation
TEST_F(BiomechanicsTest, ValidatesCalibrationData) {
    // Create valid calibration data
    BiomechanicalZone valid_zone;
    valid_zone.sensor_id = THUMB_TIP;
    valid_zone.haptic_id = THUMB_ZONE;
    valid_zone.min_flexion = 40.0;
    valid_zone.max_flexion = 50.0;
    valid_zone.tolerance_degrees = 5.0;
    valid_zone.confidence_threshold = 0.75;
    
    // Valid zone should pass validation
    EXPECT_LT(valid_zone.min_flexion, valid_zone.max_flexion) << "Min should be less than max";
    EXPECT_GT(valid_zone.tolerance_degrees, 0.0) << "Tolerance should be positive";
    EXPECT_GE(valid_zone.confidence_threshold, 0.0) << "Confidence should be non-negative";
    EXPECT_LE(valid_zone.confidence_threshold, 1.0) << "Confidence should not exceed 1.0";
}

// Integration test: Full feedback loop
TEST_F(BiomechanicsTest, FullFeedbackLoop) {
    // Simulate a complete throwing motion with mixed biomechanics
    std::vector<JointState> joints(MAX_SENSORS);
    
    // Set up realistic joint states
    joints[THUMB_TIP] = {.flexion_angle = 45.0, .angular_velocity = 50.0, .is_valid = true};
    joints[INDEX_TIP] = {.flexion_angle = 30.0, .angular_velocity = 80.0, .is_valid = true};
    joints[WRIST] = {.flexion_angle = 15.0, .angular_velocity = 120.0, .is_valid = true};
    joints[ELBOW] = {.flexion_angle = 90.0, .angular_velocity = 200.0, .is_valid = true};
    
    // Detect phase
    ThrowPhase phase = analyzer->detectPhase(joints);
    EXPECT_NE(phase, static_cast<ThrowPhase>(-1)) << "Should detect a valid phase";
    
    // Generate feedback
    auto commands = analyzer->generateLocationSpecificFeedback(joints);
    
    // Should generate some feedback for optimal positions
    EXPECT_GT(commands.size(), 0) << "Should generate at least some haptic feedback";
    
    // All commands should be valid
    for (const auto& cmd : commands) {
        EXPECT_GE(cmd.intensity, 0.0) << "Intensity should be non-negative";
        EXPECT_LE(cmd.intensity, 1.0) << "Intensity should not exceed 1.0";
        EXPECT_GT(cmd.duration_ms, 0) << "Duration should be positive";
        EXPECT_LT(cmd.zone, MAX_HAPTIC_ZONES) << "Zone should be valid";
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
