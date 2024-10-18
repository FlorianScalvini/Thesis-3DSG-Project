//
// Created by ubuntu on 11/11/22.
//

#include "mahony_ahrs.h"
#include <cmath>

Mahony::Mahony(float prop_gain, float int_gain) {
    twoKp = prop_gain; // 2 * proportional gain (Kp)
    twoKi = int_gain;  // 2 * integral gain (Ki)
    q = Quaternion(1, 0, 0, 0);
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
    anglesComputed = false;
}

void Mahony::update(float gx, float gy, float gz, float ax, float ay,
                             float az, float mx, float my, float mz, float dt) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid
    // (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        updateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q.q0 * q.q0;
        q0q1 = q.q0 * q.q1;
        q0q2 = q.q0 * q.q2;
        q0q3 = q.q0 * q.q3;
        q1q1 = q.q1 * q.q1;
        q1q2 = q.q1 * q.q2;
        q1q3 = q.q1 * q.q3;
        q2q2 = q.q2 * q.q2;
        q2q3 = q.q2 * q.q3;
        q3q3 = q.q3 * q.q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f *
             (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f *
             (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f *
             (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction
        // and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            // integral error scaled by Ki
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q.q0;
    qb = q.q1;
    qc = q.q2;
    q.q0 += (-qb * gx - qc * gy - q.q3 * gz);
    q.q1 += (qa * gx + qc * gz - q.q3 * gy);
    q.q2 += (qa * gy - qb * gz + q.q3 * gx);
    q.q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    q.normalize();
    anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// IMU algorithm update

void Mahony::updateIMU(float gx, float gy, float gz, float ax,
                                float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = q.q1 * q.q3 - q.q0 * q.q2;
        halfvy = q.q0 * q.q1 + q.q2 * q.q3;
        halfvz = q.q0 * q.q0 - 0.5f + q.q3 * q.q3;

        // Error is sum of cross product between estimated
        // and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            // integral error scaled by Ki
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx; // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt); // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q.q0;
    qb = q.q1;
    qc = q.q2;
    q.q0 += (-qb * gx - qc * gy - q.q3 * gz);
    q.q1 += (qa * gx + qc * gz - q.q3 * gy);
    q.q2 += (qa * gy - qb * gz + q.q3 * gx);
    q.q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    q.normalize();
    anglesComputed = 0;
}

void Mahony::update(float gx, float gy, float gz, float ax, float ay,
                             float az, float mx, float my, float mz) {
    update(gx, gy, gz, ax, ay, az, mx, my, mz, this->dtime);
}

void Mahony::updateIMU(float gx, float gy, float gz, float ax,
                                float ay, float az) {
    updateIMU(gx, gy, gz, ax, ay, az, this->dtime);
};

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float Mahony::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
