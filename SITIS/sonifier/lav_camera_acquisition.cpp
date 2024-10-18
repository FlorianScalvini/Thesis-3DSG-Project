//
// Created by ubuntu on 24/11/22.
//

#include "lav_camera_acquisition.h"
#include "lav_constants.h"
#include <pthread.h>


cv::Mat lavCameraAcquisition::_imgDepth;
cv::Mat lavCameraAcquisition::_imgColor;
cv::Mat lavCameraAcquisition::_imgAcquiredDepth;
cv::Mat lavCameraAcquisition::_imgAcquiredColor;
VideoCapture* lavCameraAcquisition::_captureVideo;
pthread_mutex_t  lavCameraAcquisition::mutexCamera;
bool lavCameraAcquisition::_frameHasBeenAcquired;
bool lavCameraAcquisition::_runAcqui;
long lavCameraAcquisition::timeBetweenTwoFrames;


void lavCameraAcquisition::init(){
    printf("Initialize Acquisition ...\n");
    _captureVideo = RealsenseCamera::getSingleton();
    _captureVideo->configureCamera(cv::Size(COLOR_FRAME_WIDTH, COLOR_FRAME_HEIGHT));
    mutexCamera = PTHREAD_MUTEX_INITIALIZER;
    timeBetweenTwoFrames = 0;
    _frameHasBeenAcquired = false;
    _runAcqui = false;
}

void lavCameraAcquisition::start() {
    _captureVideo->start();
    mutexCamera = PTHREAD_MUTEX_INITIALIZER;
    _frameHasBeenAcquired = false;
    _runAcqui = true;
    std::cout<<"Start camera loop\n" <<std::endl;
    acquisitionCamera();
}

void lavCameraAcquisition::release() {
    _captureVideo->release();
    _runAcqui = false;
}

void lavCameraAcquisition::acquisitionCamera()
{
    while(_runAcqui)
    {
        _captureVideo->getNextFrame(_imgAcquiredDepth, _imgAcquiredColor);
        lavCameraAcquisition::frameAcquiredAndTransfert();
    }
}

void lavCameraAcquisition::frameAcquiredAndTransfert()
{
    pthread_mutex_lock(&mutexCamera);
    if(_imgAcquiredDepth.rows != DEPTH_FRAME_HEIGHT || _imgAcquiredDepth.cols != DEPTH_FRAME_WIDTH)
        cv::resize(_imgAcquiredDepth, _imgDepth, cv::Size(DEPTH_FRAME_WIDTH,DEPTH_FRAME_HEIGHT), 0, 0, cv::INTER_NEAREST);
    else
        _imgAcquiredDepth.copyTo(_imgDepth);
    _imgAcquiredColor.copyTo(_imgColor);
    _frameHasBeenAcquired = true;
    pthread_mutex_unlock(&mutexCamera);
}

cv::Size lavCameraAcquisition::getFov() {
    return cv::Size(_captureVideo->getXFOV(), _captureVideo->getYFOV());
}

long lavCameraAcquisition::getDeltaTime() {
    return timeBetweenTwoFrames;
}

void lavCameraAcquisition::getNextFrame(cv::Mat & depth, cv::Mat & color) {
    while(!_frameHasBeenAcquired)
        usleep(1000);
    pthread_mutex_lock(&mutexCamera);
    _imgDepth.copyTo(depth);
    _imgColor.copyTo(color);
    _frameHasBeenAcquired = false;
    pthread_mutex_unlock(&mutexCamera);

}

void lavCameraAcquisition::pixelToMeter(float *pixel, Eigen::Vector3d& pointXYZ,float depth){
    float point3D[3];
    _captureVideo->pixelToWorld(pixel, point3D, depth);
    pointXYZ[0] = point3D[0] / 1000.0;
    pointXYZ[1] = point3D[1] / 1000.0;
    pointXYZ[2] = depth / 1000.0;
}

float lavCameraAcquisition::pixelToAng(int pixel, int fov, int nbPix) {
    return _captureVideo->pixelToAng(pixel, fov, nbPix);
}


void *lavCameraAcquisition::start_acquisition_inThread(void *args) {
    auto* thisPointer = (lavCameraAcquisition*) args;
    thisPointer->start();
    return nullptr;
}


void lavCameraAcquisition::start_thread_acquisition() {
    pthread_t thread_video_processing;
    pthread_create(&thread_video_processing, nullptr, start_acquisition_inThread, (void*)nullptr);
}

