//
// Created by ubuntu on 24/11/22.
//

#ifndef OUTDOORNAV_AHRS_H
#define OUTDOORNAV_AHRS_H

#include "quaternion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
struct AHRS_Data
{
    EulerAngle angle;
    cv::Point3f acc;
    AHRS_Data(EulerAngle angle, float acc) : angle(angle), acc(acc) {}
    AHRS_Data(Quaternion q, float acc) : acc(acc) {
        angle = EulerAngle(q);
    }
}
*/

class Ahrs {
public:
    virtual void update() = 0;
    virtual void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz) = 0;
    virtual void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) = 0;
    virtual void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz, float dt) = 0;
    virtual void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) = 0;
};


#endif //OUTDOORNAV_AHRS_H
