/*
 * Haptic Feedback Control for Basketball Training System
 * Standard C++ version for Visual Studio Code
 */

#ifndef HAPTIC_H
#define HAPTIC_H

#include <vector>
#include <chrono>

// Pin Definitions (for ESP32 reference)
const int HAPTIC_1_PIN = 25;  // Upper arm
const int HAPTIC_2_PIN = 26;  // Lower arm  
const int HAPTIC_3_PIN = 27;  // Wrist

// Haptic Feedback Patterns
enum HapticPattern {
    NONE,
    SINGLE_PULSE,
    DOUBLE_PULSE,
    TRIPLE_PULSE,
    CONTINUOUS,
    INCREASING,
    DECREASING,
    ALTERNATING,
    WAVE
};

// Feedback Intensity Levels
enum FeedbackIntensity {
    LIGHT = 64,
    MEDIUM = 128,
    STRONG = 255
};

// Feedback Zones
enum FeedbackZone {
    UPPER_ARM = 1,
    LOWER_ARM = 2,
    WRIST = 3,
    ALL_ZONES = 0
};

// Haptic Motor Control Structure
struct HapticMotor {
    int pin;
    int currentIntensity;
    bool isActive;
    unsigned long startTime;
    unsigned long duration;
    HapticPattern pattern;
    
    HapticMotor() : pin(0), currentIntensity(0), isActive(false), 
                   startTime(0), duration(0), pattern(NONE) {}
    
    HapticMotor(int p) : pin(p), currentIntensity(0), isActive(false), 
                        startTime(0), duration(0), pattern(NONE) {}
};

// Global Variables
extern std::vector<HapticMotor> motors;
extern bool hapticSystemEnabled;

// Function Declarations
void initHapticSystem();
void triggerHapticFeedback(int pin, int intensity, unsigned long duration);
void triggerPatternFeedback(FeedbackZone zone, HapticPattern pattern, FeedbackIntensity intensity);
void updateHapticFeedback();
void stopAllHapticFeedback();
void setHapticIntensity(int pin, int intensity);
void enableHapticSystem(bool enable);

// Pattern Functions
void executeSinglePulse(HapticMotor* motor);
void executeDoublePulse(HapticMotor* motor);
void executeTriplePulse(HapticMotor* motor);
void executeContinuous(HapticMotor* motor);
void executeIncreasing(HapticMotor* motor);
void executeDecreasing(HapticMotor* motor);
void executeAlternating();
void executeWave();

// POSITIVE REINFORCEMENT Feedback Functions
void reinforceOptimalElbow();        // Elbow in perfect zone
void reinforceOptimalWrist();        // Wrist in perfect zone  
void reinforceGoodFollowThrough();   // Follow-through consistency
void reinforcePerfectForm();         // All biomechanics optimal
void reinforceConsistentTiming();    // Consistent shot rhythm
void reinforceBodyBalance();         // Good balance during shot
void buildMuscleMemeory();          // Multi-zone success pattern

// Silence Functions (NO haptic feedback when form is poor)
void silentLearning();              // No feedback = user self-corrects

// Utility Functions
int getMotorIndex(int pin);
void setMotorPattern(int pin, HapticPattern pattern, int intensity, unsigned long duration);
bool isMotorActive(int pin);
void cleanupInactiveMotors();

// Safety Functions
void checkMotorTemperature();
void emergencyStop();
bool isSystemOverheating();

#endif // HAPTIC_H
