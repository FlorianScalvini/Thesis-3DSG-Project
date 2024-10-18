//
// Created by ubuntu on 11/11/22.
//

#include "simple_ahrs.h"
#include <cmath>

EulerAngle getOrientation(cv::Point3f mag, cv::Point3f acc)
{
    EulerAngle angles;


    // roll: Rotation around the X-axis. -180 <= roll <= 180
    // a positive roll angle is defined to be a clockwise rotation about the
    // positive X-axis
    //
    //                    y
    //      roll = atan2(---)
    //                    z
    //
    // where:  y, z are returned value from accelerometer sensor

    angles.roll = (float)atan2(acc.y, acc.z);

    // pitch: Rotation around the Y-axis. -180 <= roll <= 180
    // a positive pitch angle is defined to be a clockwise rotation about the
    // positive Y-axis
    //
    //                                 -x
    //      pitch = atan(-------------------------------)
    //                    y * sin(roll) + z * cos(roll)
    //
    // where:  x, y, z are returned value from accelerometer sensor

    if (acc.y * sin(angles.roll) + acc.z * cos(angles.roll) == 0)
        angles.pitch = acc.x > 0 ? (M_PI / 2) : (-M_PI / 2);
    else
        angles.pitch = (float)atan(-acc.x / (acc.y * sin(angles.roll) + acc.z * cos(angles.roll)));

    // heading: Rotation around the Z-axis. -180 <= roll <= 180
    // a positive heading angle is defined to be a clockwise rotation about the
    // positive Z-axis
    //
    //                                       z * sin(roll) - y * cos(roll)
    //   heading =
    //   atan2(--------------------------------------------------------------------------)
    //                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z *
    //                    sin(pitch) * cos(roll))
    //
    // where:  x, y, z are returned value from magnetometer sensor
    angles.yaw = (float)atan2(mag.z * sin(angles.roll) - mag.y * cos(angles.roll),
                              mag.x * cos(angles.pitch) + mag.y * sin(angles.pitch) * sin(angles.roll) +
                              mag.z * sin(angles.pitch) * cos(angles.roll));


    // Convert angular data to degree
    /*
    orientation->roll = orientation->roll * 180 / PI_F;
    orientation->pitch = orientation->pitch * 180 / PI_F;
    orientation->heading = orientation->heading * 180 / PI_F;
     */
}

void SimpleAHRS::process_accel(cv::Point3f& acc) {

    // Calculate rotation angle from accelerometer data
    acc.z = atan2(acc.y, acc.z);
    acc.x = atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z));

    if (firstAccel)
    {
        firstAccel = false;
        angles.pitch = acc.y;
        angles.roll = acc.z;
        // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
        angles.pitch = M_PI;
    }
    else
    {
        /*
        Apply Complementary Filter:
            - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
              that are steady over time, is used to cancel out drift.
            - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
        */
        angles.pitch = angles.pitch * alpha + angles.pitch * (1 - alpha);
        angles.roll = angles.roll * alpha + acc.z * (1 - alpha);
    }
}
void SimpleAHRS::process_gyro(cv::Point3f& gyr, float ts) {
    if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
    {
        firstGyro = false;
        last_ts_gyro = ts;
        return;
    }
    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    angles.pitch *= static_cast<float>(dt_gyro);
    angles.yaw *= static_cast<float>(dt_gyro);
    angles.roll *= static_cast<float>(dt_gyro);

    // Apply the calculated change of angle to the current angle (theta)
    angles.pitch -= gyr.z;
    angles.yaw -= gyr.y;
    angles.roll = gyr.x;

}

EulerAngle SimpleAHRS::getTheta(cv::Point3f gyr, cv::Point3f acc, float ts) {
    this->process_gyro(gyr, ts);
    this->process_accel(acc);
    return angles;
}

