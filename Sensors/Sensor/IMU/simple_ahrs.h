//
// Created by ubuntu on 11/11/22.
//

#ifndef OUTDOORNAV_SIMPLE_AHRS_H
#define OUTDOORNAV_SIMPLE_AHRS_H

#include "euler_angle.h"
#include "constant.h"

static EulerAngle getOrientation(cv::Point3f mag, cv::Point3f acc);

class SimpleAHRS
{
private:
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
    float alpha = 0.98f;
    bool firstGyro = true;
    bool firstAccel = true;
    void process_accel(cv::Point3f& acc);
    void process_gyro(cv::Point3f& gyr, float ts);
    EulerAngle angles;
public:
    EulerAngle getTheta(cv::Point3f gyr, cv::Point3f acc, float ts);


};

#endif //OUTDOORNAV_SIMPLE_AHRS_H
