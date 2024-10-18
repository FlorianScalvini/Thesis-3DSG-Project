//
// Created by ubuntu on 06/12/22.
//

#ifndef OUTDOORNAV_IMU_CAPTURE_H
#define OUTDOORNAV_IMU_CAPTURE_H

#define DIJON_MAG_DECLINATION "-2.22"
#include "constant.h"
#include "Common/utils.h"

struct DataIMU
{
    Quaternion q;
    cv::Point3f grav;
    cv::Point3f linAcc;
    cv::Point3f mag;
    double compass;

    DataIMU(){
        compass = 0;
        mag = cv::Point3f(0,0,0);
        linAcc = cv::Point3f(0,0,0);
        grav = cv::Point3f(0,0,0);
        q = Quaternion();
    }
};

class IMUCapture {
public:
    virtual void start(const char * file)=0; // start the capture
    virtual void release() = 0; // release the capture
    virtual void getQuaternion(Quaternion&) = 0;  // Return the quaternion
    virtual void getEuler(EulerAngle&) = 0; // Return the euler angles
    virtual void getGyrAccMag(cv::Point3f &gyr, cv::Point3f &acc, cv::Point3f &mag) = 0; // Read the ma
    virtual void getQuaternionLinAccGravity(Quaternion &quad, cv::Point3f& lin, cv::Point3f& grv) = 0;
    virtual void getAcceleration(cv::Point3f&) = 0; // Return the acceleration vector
    virtual void getMagnetic(cv::Point3f&) = 0; // Return the magnetic field vector
    virtual void getGyroscope(cv::Point3f&) = 0; // Return the gyroscope vector
    virtual void getLinearAcceleration(cv::Point3f&) = 0; // Return the linear acceleration
    virtual void getGravity(cv::Point3f&) = 0; // Return the gravity vector
    virtual float getCompass() = 0; // Return the compass in degree
    virtual void getDataIMU(DataIMU&) = 0;
    virtual void setCalibration(const char* file) = 0; // Set the calibration of the IMU
    virtual void  readRegister(unsigned char, unsigned char*, unsigned char) = 0;
protected:
    static void multiply(cv::Point3f& pt, float scalar)
    {
        pt.x = (float)(pt.x * scalar);
        pt.y = (float)(pt.y * scalar);
        pt.z = (float)(pt.z * scalar);
    }
    static void divide(cv::Point3f& pt, float scalar)
    {
        pt.x = (float)(pt.x / scalar);
        pt.y = (float)(pt.y / scalar);
        pt.z = (float)(pt.z / scalar);
    }
};


#endif //OUTDOORNAV_IMU_CAPTURE_H


