//
// Created by ubuntu on 24/11/22.
//

#ifndef SITIS_LAV_CAMERA_ACQUISITION_H
#define SITIS_LAV_CAMERA_ACQUISITION_H
#include <pthread.h>
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
    static float pixelToAng(int, int, int);
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
    static pthread_mutex_t  mutexCamera;
    static bool _frameHasBeenAcquired;
    static bool _runAcqui;
    static long timeBetweenTwoFrames;
};


#endif //SITIS_LAV_CAMERA_ACQUISITION_H
