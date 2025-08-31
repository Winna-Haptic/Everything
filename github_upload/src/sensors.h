/*
 * Sensor Definitions for Basketball Haptic Training System
 * Standard C++ version for Visual Studio Code
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <chrono>

// Data Structures
struct Vector3D {
    double x, y, z;
    
    Vector3D() : x(0), y(0), z(0) {}
    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
};

struct MotionData {
    Vector3D accel;      // Acceleration in g
    Vector3D gyro;       // Angular velocity in degrees/sec
    double magnitude;    // Acceleration magnitude
    unsigned long timestamp;
    
    MotionData() : magnitude(0), timestamp(0) {}
};

struct ShotData {
    unsigned long timestamp;
    double peakAccel;
    unsigned long duration;
    double formScore;
    Vector3D startPosition;
    Vector3D endPosition;
    std::vector<Vector3D> trajectory; // Store trajectory points
    
    ShotData() : timestamp(0), peakAccel(0), duration(0), formScore(0) {
        trajectory.reserve(100);
    }
};

struct CalibrationData {
    double avgPeakAccel;
    double avgDuration;
    double stdDevAccel;
    double stdDevDuration;
    std::vector<Vector3D> optimalTrajectory;
    bool isValid;
    
    CalibrationData() : avgPeakAccel(0), avgDuration(0), stdDevAccel(0), 
                       stdDevDuration(0), isValid(false) {
        optimalTrajectory.reserve(50);
    }
};

// Sensor Configuration
const int MPU6050_ADDRESS = 0x68;
const int SAMPLE_RATE = 100;  // Hz
const int CALIBRATION_SAMPLES = 10;

// Motion Detection Thresholds
const int MOTION_THRESHOLD = 5000;
const int SHOT_DETECTION_THRESHOLD = 15000;
const int MOTION_TIMEOUT = 1000;  // ms

// Global Variables
extern CalibrationData calibrationData;
extern MotionData lastMotionData;

// Function Declarations
void initSensors();
bool calibrateSensors();
void readMotionData(MotionData* data);
bool detectShotStart(const MotionData* data);
bool detectShotEnd(const MotionData* data);
double calculateFormScore(const MotionData* data, const CalibrationData* cal);
void recordTrajectory(ShotData* shot);
void analyzeTrajectory(ShotData* shot);
void saveCalibrationData(const CalibrationData* data);
void loadCalibrationData(CalibrationData* data);
double calculateTrajectorySimilarity(const std::vector<Vector3D>& traj1, 
                                    const std::vector<Vector3D>& traj2);

// Utility Functions
double vectorMagnitude(const Vector3D* vec);
double vectorDistance(const Vector3D* vec1, const Vector3D* vec2);
void normalizeVector(Vector3D* vec);
double dotProduct(const Vector3D* vec1, const Vector3D* vec2);
Vector3D crossProduct(const Vector3D* vec1, const Vector3D* vec2);

// Filtering Functions
void applyLowPassFilter(MotionData* data, double alpha);
void applyKalmanFilter(MotionData* data);
void removeGravity(MotionData* data);

// Calibration Functions
void startCalibration();
void addCalibrationSample(const ShotData* shot);
void finishCalibration();
bool validateCalibration();

#endif // SENSORS_H
