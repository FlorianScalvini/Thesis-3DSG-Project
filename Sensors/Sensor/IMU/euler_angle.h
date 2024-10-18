//
// Created by ubuntu on 10/11/22.
//

#ifndef OUTDOORNAV_EULERANGLE_H
#define OUTDOORNAV_EULERANGLE_H

#include "quaternion.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class EulerAngle {
public:
    EulerAngle();
    EulerAngle(double roll, double pitch, double yaw);
    EulerAngle(Quaternion q);

    cv::Point3f rotationMatrix(cv::Point3f pos);
    double roll, pitch, yaw;
};

static EulerAngle operator+(const EulerAngle &a, const EulerAngle &b)
{
    // use the Cents constructor and operator+(int, int)
    return EulerAngle(a.roll + b.roll, a.pitch + b.pitch, a.yaw + b.yaw);
}

static EulerAngle operator-(const EulerAngle &a, const EulerAngle &b)
{
    // use the Cents constructor and operator+(int, int)
    return EulerAngle(a.roll - b.roll, a.pitch - b.pitch, a.yaw - b.yaw);
}
#endif //OUTDOORNAV_EULERANGLE_H
