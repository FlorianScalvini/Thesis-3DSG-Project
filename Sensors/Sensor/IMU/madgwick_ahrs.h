//
// Created by ubuntu on 10/11/22.
//

#ifndef OUTDOORNAV_MADGWICK_H
#define OUTDOORNAV_MADGWICK_H

#include "euler_angle.h"
#include "quaternion.h"
#include "ahrs.h"

class Madgwick : Ahrs{
private:
    static float invSqrt(float x);
    Quaternion q;
    EulerAngle angle;
    float beta; // algorithm gain
    float dtime;
    bool anglesComputed = false;
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations
public:
    Madgwick();
    Madgwick(float gain);
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz) override;
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) override;
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz, float dt) override;
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az,
                   float dt) override;
    float getBeta() { return beta; }
    void setBeta(float beta) { this->beta = beta; }
    void setDt(float frequency) {this->dtime = 1.0f / frequency;}
};

#endif //OUTDOORNAV_MADGWICK_H
