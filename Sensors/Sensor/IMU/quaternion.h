//
// Created by ubuntu on 10/11/22.
//

#ifndef OUTDOORNAV_QUATERNION_H
#define OUTDOORNAV_QUATERNION_H


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Quaternion {
public:
    double q0, q1, q2, q3;
    Quaternion() : q0(0), q1(0), q2(0), q3(0) {};
    Quaternion(double q0, double q1, double q2, double q3) : q0(q0), q1(q1), q2(q2), q3(q3) {};
    Quaternion(double roll, double pitch, double yaw);


    cv::Point3f rotationMatrix(cv::Point3f pos);
    cv::Point3f rotationMatrix();
    void normalize();
    Quaternion inverse();
    Quaternion conjugate();
private:
    void toQuad(double roll, double pitch, double yaw);

};


/*
static Quaternion operator / (const Quaternion& c1, const float& c2)
{
    return Quaternion(c1.q0 / c2, c1.q1 / c2, c1.q2 / c2, c1.q3 / c2);
};
static Quaternion operator / (const float& c1, const Quaternion& c2)
{
    return Quaternion(c2.q0 / c1, c2.q1 / c1, c2.q2 / c1, c2.q3 / c1);
};

*/


#endif //OUTDOORNAV_QUATERNION_H
