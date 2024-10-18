//
// Created by Florian on 07/12/22.
//

#ifndef OUTDOORNAV_LAV_IMU_ACQUISITION_H
#define OUTDOORNAV_LAV_IMU_ACQUISITION_H
#include <pthread.h>
#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <unistd.h>
#include "lav_constants.h"
#include "Sensor/IMU/adafruit_imu_bno055.h"
#include "Sensor/IMU/adafruit_imu_bno055_i2c.h"
#include "Sensor/GPS_Device/adafruit_gps.h"
#include <fstream>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


class lavImuAcquisition {
public:
    static void init(const char* file, const char* fdi);
    static void release();
    static void acquisitionIMU();
    static cv::Point3f nextPositionXYZ();
    static float getCompass(); // Get compass value

    static void *start_acquisition_inThread(void *args);
    static void start_thread_acquisition();
    static void saveIMU();
    static void setSaveMode(bool);
    static void setSilence(bool);
    static bool openSaveFile(std::string);
    static void closeSaveFile();
    static void getNextValue(Eigen::Quaterniond&, cv::Point3f&); // Get the next IMU values
    static Quaternion getQuaternion(); // Get the orientation with the quaternion format


private:
    static bool _silence;
    static DataIMU dataIMU;
    static std::mutex _mutexIMU;
    static bool _aNewQuaternionDataHasBeenAcquired;
    static bool _aNewPositionDataHasBeenAcquired;
    static bool _aNewCompassDataHasBeenAcquired;
    static IMUCapture* _captureIMU;
    static bool _runAcc;
    static bool _saveMode;
    static cv::Point3f _motionBetweenTwoRead;
    static double timeDisplacement;
    static FILE* fp;

};

#endif //OUTDOORNAV_LAV_IMU_ACQUISITION_H
