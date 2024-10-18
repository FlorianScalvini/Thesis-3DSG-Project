//
// Created by florian on 29/12/22.
//

#ifndef OUTDOORNAV_ADAFRUIT_IMU_BNO055_I2C_H
#define OUTDOORNAV_ADAFRUIT_IMU_BNO055_I2C_H

#include "constant.h"
#include "quaternion.h"
#include "euler_angle.h"
#include "../GPIO/i2c.h"
#include "adafruit_bno055_constant.h"
#include "imu_capture.h"


class AdafruitIMUI2C : public IMUCapture{
public:
    static AdafruitIMUI2C* __singleton;
    static AdafruitIMUI2C* getSingleton();
    AdafruitIMUI2C() = default;
    ~AdafruitIMUI2C();
    void start(const char*) override;
    void release() override;
    char setMode(modeBNO055);
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
    void getGravity(cv::Point3f&);
    char getTemperature();
    bool isFullyCalibrated();
    char getCalibration(unsigned char *system, unsigned char *gyro, unsigned char *accel, unsigned char *mag);

    bool getSensorOffsets(unsigned char *calibData);
    bool getSensorOffsets(offsetsBNO055 &offsets_type);
    void setCalibration(const char *file) override;
    char setSensorOffsets(const unsigned char  *calibData);
    char setSensorOffsets(const offsetsBNO055 &offsets_type);

    char setGyroRange(GYRO_DPS);
    char setAccRange(ACC_G);
    char setGyroRate(GYRO_RATE);
    char setMagRate(MAG_RATE);
    char setAccRate(ACC_RATE);
    void getDataIMU(DataIMU &) override;
    void readRegister(unsigned char, unsigned char*, unsigned char=1);
    void getSystemStatus(unsigned char*, unsigned char*, unsigned char*);
private:
    void bufferToPoint3f(const unsigned char buffer[6], cv::Point3f& pt);
    void reset();
    char writeRegister(unsigned char, unsigned char);
    unsigned char readRegister(unsigned char addr_reg);
    char getValue(Register reg, cv::Point3f&);
    void readCalibrationFile(const char*, unsigned char*, int);
    cv::Point3f setRadiusAcc();
    cv::Point3f getRadiusMag();
    cv::Point3f setRadiusMag();
    modeBNO055 _mode;
    I2C *i2c{};
};



#endif //OUTDOORNAV_ADAFRUIT_IMU_BNO055_I2C_H
