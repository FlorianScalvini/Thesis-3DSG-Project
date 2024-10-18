//
// Created by ubuntu on 07/12/22.
//

#include "lav_imu_acquisition.h"
#include <chrono>

bool lavImuAcquisition::_runAcc = false;
bool lavImuAcquisition::_saveMode = false;
bool lavImuAcquisition::_silence;
std::mutex lavImuAcquisition::_mutexIMU;
bool lavImuAcquisition::_aNewCompassDataHasBeenAcquired;
bool lavImuAcquisition::_aNewPositionDataHasBeenAcquired;
bool lavImuAcquisition::_aNewQuaternionDataHasBeenAcquired;
IMUCapture* lavImuAcquisition::_captureIMU;
DataIMU lavImuAcquisition::dataIMU;
cv::Point3f lavImuAcquisition::_motionBetweenTwoRead;
double lavImuAcquisition::timeDisplacement;
FILE* lavImuAcquisition::fp;

void lavImuAcquisition::init(const char* file, const char* fdi)
{
    _captureIMU = new AdafruitIMU();
    _captureIMU->start(fdi);
    //_captureIMU->setCalibration(file);
    _runAcc = true;
    _saveMode = false;
    _silence = false;
    _aNewCompassDataHasBeenAcquired = false;
    _aNewPositionDataHasBeenAcquired = false;
    _aNewQuaternionDataHasBeenAcquired = false;
    _motionBetweenTwoRead = cv::Point3f(0,0,0);
    _mutexIMU.unlock();
    timeDisplacement = 0;
}

void lavImuAcquisition::release() {
    lavImuAcquisition::_runAcc = false;
    _captureIMU->release();
}

/*
 * Open the file descriptor to record the IMU data
 */
bool lavImuAcquisition::openSaveFile(std::string dir)
{
    std::string path = dir + "/IMUData.txt";
    fp = fopen(path.c_str(), "w+");
    if(!fp) { // file couldn't be opened
        fprintf(stderr, "Error: IMU data file could not be opened");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

/*
 * Close the file descriptor
 */
void lavImuAcquisition::closeSaveFile()
{
    if(fp)
        fclose(fp);
}


void lavImuAcquisition::saveIMU()
{
    std::string sTime;
    unsigned char buffer[83];
    if(fp) {
        while(true)
        {
            _captureIMU->readRegister(0x08, buffer, 44);
            auto now = std::chrono::system_clock::now();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
            auto timer = std::chrono::system_clock::to_time_t(now);
            std::tm bt = *std::localtime(&timer);
            sTime = std::to_string(bt.tm_hour) + ';' + std::to_string(bt.tm_min) + ';' + std::to_string(bt.tm_sec) + ';' + std::to_string(ms.count());
            fprintf(fp, "%s;", sTime.c_str());
            for(int i = 0; i < 44; i++)
            {
                fprintf(fp, "%i;", buffer[i]);
            }
            fprintf(fp, "\n");
        }
           }
    else
        printf("Error curring the save of IMU data\n");
}
void lavImuAcquisition::acquisitionIMU()
{
    DataIMU tmp;
    _captureIMU->getDataIMU(tmp);
    //vector = 0.5*vector * 0.01; // Acceleration in m/s^2 to position in m with a time step of 0.01s
    _mutexIMU.lock();
    dataIMU = tmp;
    _aNewCompassDataHasBeenAcquired = true;
    _aNewPositionDataHasBeenAcquired = true;
    _aNewQuaternionDataHasBeenAcquired = true;
    _mutexIMU.unlock();
    usleep(10000);
}
/*
 * Return the next quaternion and linear acceleration values
 */
void lavImuAcquisition::getNextValue(Eigen::Quaterniond& q, cv::Point3f& p) {
    while(!_aNewQuaternionDataHasBeenAcquired or !_aNewPositionDataHasBeenAcquired)
        usleep(500);
    _mutexIMU.lock();
    q = Eigen::Quaterniond(dataIMU.q.q0, dataIMU.q.q1, dataIMU.q.q2, dataIMU.q.q3);
    p = dataIMU.linAcc;
    _aNewPositionDataHasBeenAcquired = false;
    _aNewQuaternionDataHasBeenAcquired = false;
    _mutexIMU.unlock();
}

/*
 * Return the next quaternion
 */
Quaternion lavImuAcquisition::getQuaternion() {
    Quaternion q;
    while(!_aNewQuaternionDataHasBeenAcquired)
        usleep(500);
    _mutexIMU.lock();
    q = dataIMU.q;
    _aNewQuaternionDataHasBeenAcquired = false;
    _mutexIMU.unlock();
}

/*
 * Return the next position
 */
cv::Point3f lavImuAcquisition::nextPositionXYZ() {
    cv::Point3f pt;
    while(!_aNewPositionDataHasBeenAcquired)
        usleep(500);
    _mutexIMU.lock();
    pt.x = _motionBetweenTwoRead.x;
    pt.y = _motionBetweenTwoRead.y;
    pt.z = _motionBetweenTwoRead.z;
    _motionBetweenTwoRead = {0,0,0};
    _aNewPositionDataHasBeenAcquired = false;
    _mutexIMU.unlock();
    return pt;
}

/*
 * Return the compass orientation
 */
float lavImuAcquisition::getCompass() {
    while(!_aNewCompassDataHasBeenAcquired)
        usleep(500);
    _mutexIMU.unlock();
    double heading = dataIMU.compass;
    _aNewCompassDataHasBeenAcquired = false;
    _mutexIMU.lock();
    return heading;
}


/*
 * Set the IMU Acquisition thread in Save mode
 */
void lavImuAcquisition::setSaveMode(bool mode) {
    _mutexIMU.unlock();
    _saveMode = mode;
    _mutexIMU.unlock();
}

/*
 * Set the IMU Acquisition in sleep mode
 */
void lavImuAcquisition::setSilence(bool mode) {
    _mutexIMU.unlock();
    _silence = mode;
    _mutexIMU.unlock();
}


void* lavImuAcquisition::start_acquisition_inThread(void *args) {
    _runAcc = true;
    printf("Start imu acqui\n");
    while(_runAcc)
    {
        if(!_silence)
        {
            if(_saveMode)
                saveIMU();
            else
                acquisitionIMU();
        }
    }
    usleep(500);
}


void lavImuAcquisition::start_thread_acquisition() {
    pthread_t thread_realsense_acquisition;
    pthread_create(&thread_realsense_acquisition, nullptr, start_acquisition_inThread, (void*)nullptr);
}
