//
// Created by ubuntu on 10/11/22.
//

#include "quaternion.h"
#import <cmath>
#include "constant.h"

Quaternion::Quaternion(double roll, double pitch, double yaw)
{
    this->toQuad(roll, pitch, yaw);
}
/*
Quaternion::Quaternion(EulerAngle angle)
{
    this->toQuad(angle.roll, angle.pitch, angle.yaw);
}*/


cv::Point3f Quaternion::rotationMatrix(cv::Point3f v)
{
    this->normalize();
    cv::Point3f q = cv::Point3f(q1, q2, q3);
    cv::Point3f t = (2*q).cross(v); // t = 2q x v
    return v + q0 * t + q.cross(t); // q0*t+ q*t
}

void Quaternion::toQuad(double roll, double pitch, double yaw) {

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
}

void Quaternion::normalize()
{
    double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

Quaternion Quaternion::conjugate()
{
    return  {this->q0, -this->q1, -this->q2, -this->q3};
}


Quaternion Quaternion::inverse() {
    this->normalize();
    return this->conjugate();
}