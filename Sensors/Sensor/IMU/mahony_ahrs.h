//
// Created by ubuntu on 11/11/22.
//

#ifndef OUTDOORNAV_MAHONY_AHRS_H
#define OUTDOORNAV_MAHONY_AHRS_H

#include "constant.h"
#include "quaternion.h"
#include "euler_angle.h"
#include "ahrs.h"



//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony : Ahrs{
private:
    float twoKp; // 2 * proportional gain (Kp)
    float twoKi; // 2 * integral gain (Ki)
    Quaternion q;
    float integralFBx, integralFBy,
            integralFBz; // integral error terms scaled by Ki
    float dtime;
    bool anglesComputed = false;
    static float invSqrt(float x);
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations

public:
    Mahony();
    Mahony(float kp, float ki);
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz) override;
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) override;
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz, float dt) override;
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az,
                   float dt) override;
    float getKp() { return twoKp / 2.0f; }
    void setKp(float Kp) { twoKp = 2.0f * Kp; }
    float getKi() { return twoKi / 2.0f; }
    void setKi(float Ki) { twoKi = 2.0f * Ki; }
    void setDt(float frequency) {this->dtime = 1.0f / frequency;}
};

#endif //OUTDOORNAV_MAHONY_AHRS_H
