/*
 * Data Logger for Basketball Haptic Training System
 * Standard C++ version for Visual Studio Code
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <string>
#include <vector>
#include <fstream>
#include "sensors.h"

// File Definitions
const std::string SHOT_DATA_FILE = "shot_data.csv";
const std::string CALIBRATION_FILE = "calibration.dat";
const std::string PERFORMANCE_FILE = "performance.csv";
const std::string CONFIG_FILE = "config.txt";

// Data Logging Configuration
const int MAX_SHOTS_PER_SESSION = 100;
const int MAX_TRAJECTORY_POINTS = 100;
const int LOG_BUFFER_SIZE = 512;

// Performance Metrics
struct PerformanceMetrics {
    int totalShots;
    double averageScore;
    double bestScore;
    double consistencyScore;
    unsigned long totalTrainingTime;
    int improvementTrend;
    double accuracyRate;
    
    PerformanceMetrics() : totalShots(0), averageScore(0), bestScore(0), 
                          consistencyScore(0), totalTrainingTime(0), 
                          improvementTrend(0), accuracyRate(0) {}
};

// Shot Statistics
struct ShotStatistics {
    unsigned long timestamp;
    double formScore;
    double peakAccel;
    unsigned long duration;
    bool wasSuccessful;
    std::string notes;
    
    ShotStatistics() : timestamp(0), formScore(0), peakAccel(0), 
                      duration(0), wasSuccessful(false) {}
};

// Data Logging Modes
enum LoggingMode {
    LOG_NONE,
    LOG_FILE_ONLY,
    LOG_CLOUD_ONLY,
    LOG_BOTH
};

// Global Variables
extern LoggingMode currentLoggingMode;
extern PerformanceMetrics performanceMetrics;
extern bool fileSystemAvailable;

// Function Declarations
void initDataLogger();
bool initFileSystem();
void logShotData(const ShotData* shot);
void logCalibrationData(const CalibrationData* data);
void logPerformanceMetrics(const PerformanceMetrics* metrics);
void saveShotToFile(const ShotData* shot);
void saveCalibrationToFile(const CalibrationData* data);
void loadCalibrationFromFile(CalibrationData* data);
void exportDataToCSV();
void clearAllData();

// Performance Analysis Functions
void calculatePerformanceMetrics();
double getAverageScore();
double getBestScore();
double getConsistencyScore();
int getImprovementTrend();
void updatePerformanceMetrics(const ShotData* shot);

// Data Export Functions
void exportShotDataToCSV();
void exportPerformanceToCSV();
void sendDataToCloud();
void backupDataToCloud();

// Utility Functions
bool isFileSystemMounted();
unsigned long getFreeSpace();
void formatFileSystem();
void listFiles();
void deleteOldData(int daysOld);

// Cloud Integration Functions
void initCloudConnection();
bool uploadToCloud(const std::string& filename);
bool downloadFromCloud(const std::string& filename);
void syncDataWithCloud();

// Data Validation Functions
bool validateShotData(const ShotData* shot);
bool validateCalibrationData(const CalibrationData* data);
void repairCorruptedData();

// Memory Management
void optimizeStorage();
void compressOldData();
void cleanupTempFiles();

#endif // DATA_LOGGER_H
