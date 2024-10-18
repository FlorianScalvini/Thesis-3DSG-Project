//
// Created by ubuntu on 10/11/22.
//

#include "euler_angle.h"
#include <cmath>

EulerAngle::EulerAngle() {
    roll = 0;
    yaw = 0;
    pitch = 0;
}

EulerAngle::EulerAngle(Quaternion q) {
    double sinr_cosp = 2 * (q.q0 * q.q1 + q.q2 * q.q3);
    double cosr_cosp = 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.q0 * q.q2 - q.q3 * q.q1);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.q0 * q.q3 + q.q1 * q.q2);
    double cosy_cosp = 1 - 2 * (q.q2 * q.q2 + q.q3 * q.q3);
    yaw = std::atan2(siny_cosp, cosy_cosp);

}

EulerAngle::EulerAngle(double roll, double pitch, double yaw) {
    this->pitch = pitch;
    this->yaw = yaw;
    this->roll = roll;
}

cv::Point3f EulerAngle::rotationMatrix(cv::Point3f pos) {
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    cv::Mat point = (cv::Mat_<double>(3,1) <<
            pos.x,pos.y,pos.z);

    cv::Mat rotmat = (cv::Mat_<double>(3,3) <<
            cp*cy, -cr*sy + sr*sp*cy,  sr*sy + cr*sp*cy,
            cp*sy,  cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy,
            -sp  ,  sr*cp           ,  cr*cp);
    point = rotmat * point;
    return cv::Point3f(point.at<double>(0), point.at<double>(1), point.at<double>(2));
}