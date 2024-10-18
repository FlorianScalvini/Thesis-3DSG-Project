//
// Created by Florian on 24/11/22.
//

#ifndef OUTDOORNAV_LAV_CAMERA_ACQUISITION_H
#define OUTDOORNAV_LAV_CAMERA_ACQUISITION_H
#include <pthread.h>
#include "Sensor/GPS_Device/adafruit_gps.h"
#include "Sensor/IMU/adafruit_imu_bno055.h"
#include "Camera/video_capture.h"
#include "Camera/realsense_camera.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ctime>
#include <iostream>
#define UART_GPS "/dev/ttyUSB1"
#define UART_IMU "/dev/ttyUSB0"


class lavCameraAcquisition
{
public:
    static void init();
    static void start();
    static void release();
    static void getNextFrame(cv::Mat&, cv::Mat&);
    static void pixelToMeter(float *, Eigen::Vector3d&, float);
    static long getDeltaTime();
    static cv::Size getFov();

    static void* start_acquisition_inThread(void* args);
    static void start_thread_acquisition();

private:
    static void acquisitionCamera();
    static void frameAcquiredAndTransfert();

    static cv::Mat _imgDepth;
    static cv::Mat _imgColor;
    static cv::Mat _imgAcquiredDepth;
    static cv::Mat _imgAcquiredColor;
    static VideoCapture* _captureVideo;
    static std::mutex  _mutexCamera;
    static bool _frameHasBeenAcquired;
    static bool _runAcqui;
    static long timeBetweenTwoFrames;
};


#endif //OUTDOORNAV_LAV_CAMERA_ACQUISITION_H
