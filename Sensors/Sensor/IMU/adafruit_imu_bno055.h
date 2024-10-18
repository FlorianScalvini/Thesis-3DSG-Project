//
// Created by ubuntu on 24/11/22.
//

#ifndef OUTDOORNAV_ADAFRUIT_IMU_BNO055_H
#define OUTDOORNAV_ADAFRUIT_IMU_BNO055_H

#include "constant.h"
#include "quaternion.h"
#include "euler_angle.h"

#include "../GPIO/uart.h"
#include "../GPIO/i2c.h"
#include "adafruit_bno055_constant.h"
#include "imu_capture.h"


class AdafruitIMU : public IMUCapture{
public:
    static AdafruitIMU* __singleton;
    static AdafruitIMU* getSingleton();
    AdafruitIMU() = default;
    ~AdafruitIMU();
    void start(const char*) override;
    void release() override;
    void setMode(modeBNO055);
    int getMode();
    void setNormalMode();
    void setSuspendMode();


    void getGyrAccMag(cv::Point3f&, cv::Point3f&, cv::Point3f&) override;
    void getQuaternionLinAccGravity(Quaternion &quad, cv::Point3f& lin, cv::Point3f& grv) override;
    void getAcceleration(cv::Point3f&) override;
    void getMagnetic(cv::Point3f&) override;
    void getQuaternion(Quaternion&) override;
    void getGyroscope(cv::Point3f&) override;
    void getEuler(EulerAngle&) override;
    float getCompass() override;
    void getLinearAcceleration(cv::Point3f&) override;
    void getDataIMU(DataIMU&) override;
    void getGravity(cv::Point3f&) override;
    char getTemperature();
    bool isFullyCalibrated();
    void getCalibration(unsigned char *system, unsigned char *gyro, unsigned char *accel, unsigned char *mag);

    bool getSensorOffsets(unsigned char *calibData);
    bool getSensorOffsets(offsetsBNO055 &offsets_type);
    void setCalibration(const char *file) override;
    void setSensorOffsets(const unsigned char  *calibData);
    void setSensorOffsets(const offsetsBNO055 &offsets_type);

    char setGyroRange(GYRO_DPS);
    void setAccRange(ACC_G);
    char setGyroRate(GYRO_RATE);
    char setMagRate(MAG_RATE);
    char setAccRate(ACC_RATE);

    void getSystemStatus(unsigned char*, unsigned char*, unsigned char*);


    void readRegister(unsigned char, unsigned char*, unsigned char=1);
private:

    void reset();
    void writeRegister(unsigned char, unsigned char, bool=true);
    unsigned char readRegister(unsigned char addr_reg);
    char getValue(Register reg, cv::Point3f&);
    static void readCalibrationFile(const char*, unsigned char*, int);
    static void bufferToPoint3f(const unsigned char[6], cv::Point3f&);
    cv::Point3f setRadiusAcc();
    cv::Point3f getRadiusMag();
    cv::Point3f setRadiusMag();
    modeBNO055 _mode;
    UART *serial{};
};




#endif //OUTDOORNAV_ADAFRUIT_IMU_BNO055_H
