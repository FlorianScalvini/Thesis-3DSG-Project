//
// Created by florian on 29/12/22.
//

#include "adafruit_imu_bno055_i2c.h"
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <fstream>
#include <cstdlib>
#include <istream>
#include <cmath>

AdafruitIMUI2C* AdafruitIMUI2C::__singleton = nullptr;

AdafruitIMUI2C* AdafruitIMUI2C::getSingleton()
{
    if (__singleton ==nullptr) {
        __singleton = new AdafruitIMUI2C();
    }
    return __singleton;
}

void AdafruitIMUI2C::start(const char * file) {

    i2c = new I2C();
    if(i2c->openFDI(file, 0x28))
    {
        std::cout<<"Serial error";
        return;
    }
    this->reset();
    this->setNormalMode();
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    this->writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x00);
    this->writeRegister(BNO055_PWR_MODE_ADDR, Powermode_BNO055::POWER_MODE_NORMAL);
    this->writeRegister(BNO055_ACCEL_CONFIG_ADDR, ACC_RATE::ACCEL_7_81HZ);

    usleep(10000);
    setMode(modeBNO055::OPERATION_MODE_NDOF);
    usleep(10000);
    std::cout<<"IMU's initialization done"<<std::endl;
}


void AdafruitIMUI2C::release() {
    delete i2c;
    i2c = nullptr;
}

AdafruitIMUI2C::~AdafruitIMUI2C() {
    this->release();
}

void AdafruitIMUI2C::getQuaternionLinAccGravity(Quaternion &quad, cv::Point3f& lin, cv::Point3f& grv) {
    this->getQuaternion(quad);
    this->getGravity(grv);
    this->getLinearAcceleration(lin);
}

void AdafruitIMUI2C::getGyrAccMag(cv::Point3f& gyr, cv::Point3f& acc, cv::Point3f& mag) {
    this->getGyroscope(gyr);
    this->getAcceleration(acc);
    this->getMagnetic(mag);
}

void AdafruitIMUI2C::reset() {
    setMode(modeBNO055::OPERATION_MODE_CONFIG);
    usleep(10000);
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x20);
    usleep(1000000);
}

char AdafruitIMUI2C::setMode(modeBNO055 mode) {
    _mode = mode;
    char err = writeRegister(BNO055_OPR_MODE_ADDR, mode);
    usleep(30000);
    return err;
}

int AdafruitIMUI2C::getMode() {

    unsigned char val;
    val = this->readRegister(BNO055_OPR_MODE_ADDR);
}


void AdafruitIMUI2C::setNormalMode() {
    writeRegister(BNO055_PWR_MODE_ADDR, 0x00);

    usleep(20000);
}

void AdafruitIMUI2C::setSuspendMode() {
    writeRegister(BNO055_PWR_MODE_ADDR, 0x02);
    /* Set the requested operating mode (see section 3.3) */
    usleep(20000);
}

bool AdafruitIMUI2C::isFullyCalibrated() {
    unsigned char system, gyro, accel, mag;
    if(this->getCalibration(&system, &gyro, &accel, &mag))
        return false;
    else
    {
        switch (_mode) {
            case OPERATION_MODE_ACCONLY:
                return (accel == 3);
            case OPERATION_MODE_MAGONLY:
                return (mag == 3);
            case OPERATION_MODE_GYRONLY:
            case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
                return (gyro == 3);
            case OPERATION_MODE_ACCMAG:
            case OPERATION_MODE_COMPASS:
                return (accel == 3 && mag == 3);
            case OPERATION_MODE_ACCGYRO:
            case OPERATION_MODE_IMUPLUS:
                return (accel == 3 && gyro == 3);
            case OPERATION_MODE_MAGGYRO:
                return (mag == 3 && gyro == 3);
            default:
                return (gyro == 3 && accel == 3 && mag == 3);
        }
    }

}

char AdafruitIMUI2C::getCalibration(unsigned char *system, unsigned char *gyro, unsigned char *accel, unsigned char *mag) {
    unsigned char calData;
    this->readRegister(BNO055_CALIB_STAT_ADDR, &calData, 1);
    if (system != nullptr) {
        *system = (calData >> 6) & 0x03;
    }
    if (gyro != nullptr) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != nullptr) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != nullptr) {
        *mag = calData & 0x03;
    }
    return EXIT_SUCCESS;
}



void AdafruitIMUI2C::getDataIMU(DataIMU& output) {
    unsigned char buffer[44];
    this->getQuaternionLinAccGravity(output.q, output.linAcc, output.grav);
    this->getMagnetic(output.mag);

    double phi = atan2(output.grav.y / 9.8, output.grav.z / 9.8); // Psi = atan2(gy, gz)
    double theta = atan2(output.grav.x / 9.8, output.grav.z / 9.8); // Theta = atan2(-gx, gy * sin(phi) + gz * cos(phi))

    if(theta > M_PI / 2.0)
        theta = M_PI - theta;
    else if(theta < -M_PI / 2.0)
        theta = -M_PI - theta;

    double cTheta = cos(theta);
    double sTheta = sin(theta);
    double sPhi = sin(phi);
    double cPhi = cos(phi);
    if(cTheta < 0)
        cTheta = -cTheta;
    /*
     * Calculate the heading from the magnetometer data with a rotation corresponding to the current orientation Theta & Phi.
     * Heading = atan2(my                                                                                            * cos(phi) - mz * sin(phi), mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi)
     */
    output.compass= atan2(output.mag.z * sPhi + output.mag.y * cPhi, output.mag.x * cTheta - output.mag.y * sTheta * sPhi + output.mag.z * sTheta * cPhi);
    output.compass = output.compass * 180.0 / M_PI; // Convert to degrees
    output.compass -= convertDegMinToDecDeg(DIJON_MAG_DECLINATION);
}


void AdafruitIMUI2C::getAcceleration(cv::Point3f& pt) {
    this->getValue(Register::BNO055_ACCEL_DATA_X_LSB_ADDR, pt);
    /* 1m/s^2 = 100 LSB */
    IMUCapture::divide(pt, 100.0);
}
/*
 *  Calculate the heading from the magnetometer data and the gravity vector from the accelerometer data.
 */
float AdafruitIMUI2C::getCompass() {
    cv::Point3f pt_mag, pt_grav;
    this->getMagnetic(pt_mag);
    this->getGravity(pt_grav);
    double phi = atan2(pt_grav.y, pt_grav.z); // Psi = atan2(gy, gz)
    double theta = atan2(-pt_grav.x, pt_grav.y * sin(phi) + pt_grav.z * cos(phi)); // Theta = atan2(-gx, gy * sin(phi) + gz * cos(phi))

    if(theta > M_PI / 2.0)
        theta = M_PI - theta;
    else if(theta < -M_PI / 2.0)
        theta = -M_PI - theta;

    double cTheta = cos(theta);
    double sTheta = sin(theta);
    double sPhi = sin(phi);
    double cPhi = cos(phi);
    if(cTheta < 0)
        cTheta = -cTheta;
    /*
     * Calculate the heading from the magnetometer data with a rotation corresponding to the current orientation Theta & Phi.
     * Heading = atan2(my                                                                                            * cos(phi) - mz * sin(phi), mx * cos(theta) + my * sin(theta) * sin(phi) + mz * sin(theta) * cos(phi)
     */
    double heading = atan2(pt_mag.z * sPhi - pt_mag.y * cPhi, pt_mag.x * cTheta + pt_mag.y * sTheta * sPhi + pt_mag.z * sTheta * cPhi);
    heading = heading * 180.0 / M_PI; // Convert to degrees
    heading -= convertDegMinToDecDeg(DIJON_MAG_DECLINATION);
    return (float)heading;
}

/*
 * Return the gravity vector
 */
void AdafruitIMUI2C::getGravity(cv::Point3f& pt) {
    this->getValue(Register::BNO055_GRAVITY_DATA_X_LSB_ADDR, pt);
    /* 1m/s^2 = 100 LSB */
    IMUCapture::divide(pt, 100.0);
}
/*
 * Return the Linear Acceleration vector
 */
void AdafruitIMUI2C::getLinearAcceleration(cv::Point3f& pt) {
    this->getValue(Register::BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, pt);
    /* 1m/s^2 = 100 LSB */
    IMUCapture::divide(pt, 100.0);
}


void AdafruitIMUI2C::getMagnetic(cv::Point3f& pt) {
    this->getValue(Register::BNO055_MAG_DATA_X_LSB_ADDR, pt);
    /* 1uT = 16 LSB */
    IMUCapture::divide(pt, 16.0);
}

void AdafruitIMUI2C::getGyroscope(cv::Point3f& pt) {
    this->getValue(Register::BNO055_GYRO_DATA_X_LSB_ADDR, pt);
    /* 1 degrees = 16 LSB  && 1 degrees =  0.0174533 rad*/
    IMUCapture::multiply(pt,0.001090830782496456);
}

/*
 * Return the orientation in the quaternion form
 */
void AdafruitIMUI2C::getQuaternion(Quaternion &quad) {
    unsigned char buffer[8];
    /* Read vector data (8 bytes) */
    this->readRegister(Register::BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 4);
    this->readRegister(Register::BNO055_QUATERNION_DATA_Y_LSB_ADDR, buffer + 4, 4);
    /*!
     * Assign to Quaternion
     * See
     * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
     * 3.6.5.5 Orientation (Quaternion)
     */
    const double scale = (1.0 / (1 << 14));
    quad.q0 = (short)((((unsigned short)buffer[1]) << 8) | ((unsigned short)buffer[0])) * scale;
    quad.q1 = (short)((((unsigned short)buffer[3]) << 8) | ((unsigned short)buffer[2])) * scale;
    quad.q2 = (short)((((unsigned short)buffer[5]) << 8) | ((unsigned short)buffer[4])) * scale;
    quad.q3 = (short)((((unsigned short)buffer[7]) << 8) | ((unsigned short)buffer[6])) * scale;
    //printf("Read return : 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X\n", buffer[0],  buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
}

void AdafruitIMUI2C::getEuler(EulerAngle& angle) {
    cv::Point3f pt;
    this->getValue(Register::BNO055_EULER_H_LSB_ADDR, pt);
    /* 1 degree = 16 LSB */
    IMUCapture::multiply(pt, 0.001090830782496456);
}

/*
 * Return the temperature in degree
 */
char AdafruitIMUI2C::getTemperature() {
    unsigned char temp;
    temp = readRegister(Register::BNO055_TEMP_ADDR);
    return (char)temp;
}

/*
 *  Return a vector of 3 elements read from the IMU at the given address
 */
char AdafruitIMUI2C::getValue(Register reg, cv::Point3f& pt) {
    unsigned char buffer[6];
    /* Read vector data (6 bytes) */
    this->readRegister(reg, buffer, 6);
    bufferToPoint3f(buffer, pt);

    //printf("Read return : 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X\n",buffer[0],  buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
    //printf("Read return : %i %i %i\n", x,  y, z);
    return EXIT_SUCCESS;
}

void AdafruitIMUI2C::bufferToPoint3f(const unsigned char buffer[6], cv::Point3f& pt) {
    pt.x = (float)((short)(((short) buffer[0]) | (((short) buffer[1]) << 8)));
    pt.y = (float)((short)(((short) buffer[2]) | (((short) buffer[3]) << 8)));
    pt.z = (float)((short)(((short) buffer[4]) | (((short) buffer[5]) << 8)));
    //printf("Read return : 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X\n",buffer[0],  buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
    //printf("Read return : %i %i %i\n", x,  y, z);
}





/*!
 *  @brief  Reads the sensor's offset registers into a byte array
 *  @param  calibData
 *          Calibration offset (buffer size should be 22)
 *  @return true if read is successful
 */
bool AdafruitIMUI2C::getSensorOffsets(unsigned char *buffer) {
    if (isFullyCalibrated()) {
        modeBNO055 lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        this->readRegister(ACCEL_OFFSET_X_LSB_ADDR, buffer, 6);
        this->readRegister(MAG_OFFSET_X_LSB_ADDR, buffer+6, 6);
        this->readRegister(GYRO_OFFSET_X_LSB_ADDR, buffer+12, 6);
        this->readRegister(ACCEL_RADIUS_LSB_ADDR, buffer+18, 2);
        this->readRegister(MAG_RADIUS_LSB_ADDR, buffer+20, 2);
        setMode(lastMode);
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

/*!
 *  @brief  Reads the sensor's offset registers into an offset struct
 *  @param  offsets_type
 *          type of offsets
 *  @return true if read is successful
 */
bool AdafruitIMUI2C::getSensorOffsets(offsetsBNO055 &offsets_type) {
    if (isFullyCalibrated()) {
        modeBNO055 lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        usleep(25000);

        /* Accel offset range depends on the G-range:
           +/-2g  = +/- 2000 mg
           +/-4g  = +/- 4000 mg
           +/-8g  = +/- 8000 mg
           +/-1§g = +/- 16000 mg */
        offsets_type.accel_offset_x = (readRegister(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (readRegister(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (readRegister(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (readRegister(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (readRegister(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (readRegister(ACCEL_OFFSET_Z_LSB_ADDR));

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets_type.mag_offset_x =
                (readRegister(MAG_OFFSET_X_MSB_ADDR) << 8) | (readRegister(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y =
                (readRegister(MAG_OFFSET_Y_MSB_ADDR) << 8) | (readRegister(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z =
                (readRegister(MAG_OFFSET_Z_MSB_ADDR) << 8) | (readRegister(MAG_OFFSET_Z_LSB_ADDR));

        /* Gyro offset range depends on the DPS range:
          2000 dps = +/- 32000 LSB
          1000 dps = +/- 16000 LSB
           500 dps = +/- 8000 LSB
           250 dps = +/- 4000 LSB
           125 dps = +/- 2000 LSB
           ... where 1 DPS = 16 LSB */
        offsets_type.gyro_offset_x = (readRegister(GYRO_OFFSET_X_MSB_ADDR) << 8) | (readRegister(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y = (readRegister(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (readRegister(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z = (readRegister(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (readRegister(GYRO_OFFSET_Z_LSB_ADDR));

        /* Accelerometer radius = +/- 1000 LSB */
        offsets_type.accel_radius = (readRegister(ACCEL_RADIUS_MSB_ADDR) << 8) | (readRegister(ACCEL_RADIUS_LSB_ADDR));

        /* Magnetometer radius = +/- 960 LSB */
        offsets_type.mag_radius = (readRegister(MAG_RADIUS_MSB_ADDR) << 8) | (readRegister(MAG_RADIUS_LSB_ADDR));

        setMode(lastMode);
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

/*!
 *  @brief  Writes an array of calibration values to the sensor's offset
 *  @param  calibData
 *          calibration data
 */
char AdafruitIMUI2C::setSensorOffsets(const unsigned char *calibData) {
    modeBNO055 lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    usleep(25000);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    signed char err;
    /* A writeLen() would make this much cleaner */
    err = writeRegister(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    err = (char)(err | writeRegister(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]));

    err = (char)(err | writeRegister(MAG_OFFSET_X_LSB_ADDR, calibData[6]));
    err = (char)(err | writeRegister(MAG_OFFSET_X_MSB_ADDR, calibData[7]));
    err = (char)(err | writeRegister(MAG_OFFSET_Y_LSB_ADDR, calibData[8]));
    err = (char)(err | writeRegister(MAG_OFFSET_Y_MSB_ADDR, calibData[9]));
    err = (char)(err | writeRegister(MAG_OFFSET_Z_LSB_ADDR, calibData[10]));
    err = (char)(err | writeRegister(MAG_OFFSET_Z_MSB_ADDR, calibData[11]));

    err = (char)(err | writeRegister(GYRO_OFFSET_X_LSB_ADDR, calibData[12]));
    err = (char)(err | writeRegister(GYRO_OFFSET_X_MSB_ADDR, calibData[13]));
    err = (char)(err | writeRegister(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]));
    err = (char)(err | writeRegister(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]));
    err = (char)(err | writeRegister(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]));
    err = (char)(err | writeRegister(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]));

    err = (char)(err | writeRegister(ACCEL_RADIUS_LSB_ADDR, calibData[18]));
    err = (char)(err | writeRegister(ACCEL_RADIUS_MSB_ADDR, calibData[19]));

    err = (char)(err | writeRegister(MAG_RADIUS_LSB_ADDR, calibData[20]));
    err = (char)(err | writeRegister(MAG_RADIUS_MSB_ADDR, calibData[21]));

    err = (char)( err | setMode(lastMode));
    return err;
}

void AdafruitIMUI2C::setCalibration(const char *file) {
    unsigned char buffer[22];
    this->readCalibrationFile(file, buffer, 22);
    this->setSensorOffsets(buffer);
}

/*!
 *  @brief  Writes to the sensor's offset registers from an offset struct
 *  @param  offsets_type
 *          accel_offset_x = acceleration offset x
 *          accel_offset_y = acceleration offset y
 *          accel_offset_z = acceleration offset z
 *
 *          mag_offset_x   = magnetometer offset x
 *          mag_offset_y   = magnetometer offset y
 *          mag_offset_z   = magnetometer offset z
 *
 *          gyro_offset_x  = gyroscrope offset x
 *          gyro_offset_y  = gyroscrope offset y
 *          gyro_offset_z  = gyroscrope offset z
 */
char AdafruitIMUI2C::setSensorOffsets(const offsetsBNO055 &offsets_type) {
    modeBNO055 lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    usleep(25000);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    char err;
    err = writeRegister(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    err = (char)(err | writeRegister(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF));
    err = (char)(err | writeRegister(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF));

    err = (char)(err | writeRegister(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF));
    err = (char)(err | writeRegister(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF));
    err = (char)(err | writeRegister(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF));
    err = (char)(err | writeRegister(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF));
    err = (char)(err | writeRegister(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF));
    err = (char)(err | writeRegister(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF));

    err = (char)(err | writeRegister(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF));
    err = (char)(err | writeRegister(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF));
    err = (char)(err | writeRegister(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF));
    err = (char)(err | writeRegister(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF));
    err = (char)(err | writeRegister(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF));
    err = (char)(err | writeRegister(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF));

    err = (char)(err | writeRegister(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF));
    err = (char)(err | writeRegister(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF));

    err = (char)(err | writeRegister(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF));
    err = (char)(err | writeRegister(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF));

    err = (char)(err | setMode(lastMode));
    return err;
}

char AdafruitIMUI2C::setAccRange(ACC_G reg = ACCEL_4G) {
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value = readRegister(BNO055_ACCEL_CONFIG_ADDR);
    unsigned char masked_value = 0b11111100 & value;
    this->writeRegister(BNO055_ACCEL_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

char AdafruitIMUI2C::setAccRate(ACC_RATE reg = ACCEL_62_5HZ) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value = readRegister(BNO055_ACCEL_CONFIG_ADDR);
    unsigned char masked_value = 0b11100011 & value;
    this->writeRegister(BNO055_ACCEL_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

char AdafruitIMUI2C::setMagRate(MAG_RATE reg = MAGNET_20HZ) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value = readRegister(BNO055_MAG_CONFIG_ADDR);
    unsigned char masked_value = 0b01111000 & value;
    this->writeRegister(BNO055_ACCEL_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

char AdafruitIMUI2C::setGyroRange(GYRO_DPS reg = GYRO_2000_DPS) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value = readRegister(BNO055_GYRO_0_CONFIG_ADDR);
    unsigned char masked_value = 0b00111000 & value;
    this->writeRegister(BNO055_GYRO_0_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

char AdafruitIMUI2C::setGyroRate(GYRO_RATE reg = GYRO_32HZ) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value = readRegister(BNO055_GYRO_0_CONFIG_ADDR);
    unsigned char masked_value = 0b00000111 & value;
    this->writeRegister(BNO055_GYRO_0_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

void AdafruitIMUI2C::getSystemStatus(unsigned char * system_status, unsigned char *self_test_result, unsigned char *system_error) {
    writeRegister(BNO055_PAGE_ID_ADDR, 0);
    /* System Status (see section 4.3.58)
       0 = Idle
       1 = System Error
       2 = Initializing Peripherals
       3 = System Iniitalization
       4 = Executing Self-Test
       5 = Sensor fusio algorithm running
       6 = System running without fusion algorithms
     */

    if (system_status != nullptr)
        *system_status = readRegister(BNO055_SYS_STAT_ADDR);

    /* Self Test Results
       1 = test passed, 0 = test failed
       Bit 0 = Accelerometer self test
       Bit 1 = Magnetometer self test
       Bit 2 = Gyroscope self test
       Bit 3 = MCU self test
       0x0F = all good!
     */

    if (self_test_result != nullptr)
        *self_test_result = readRegister(BNO055_SELFTEST_RESULT_ADDR);

    /* System Error (see section 4.3.59)
       0 = No error
       1 = Peripheral initialization error
       2 = System initialization error
       3 = Self test result failed
       4 = Register map value out of range
       5 = Register map address out of range
       6 = Register map write error
       7 = BNO low power mode not available for selected operat ion mode
       8 = Accelerometer power mode not available
       9 = Fusion algorithm configuration error
       A = Sensor configuration error
     */

    if (system_error != nullptr)
        *system_error = readRegister(BNO055_SYS_ERR_ADDR);
    usleep(200000);
}



char AdafruitIMUI2C::writeRegister(unsigned char addr_reg, unsigned char value) {
    i2c->writeReg(addr_reg, (char*)&value, 1);
    return EXIT_SUCCESS;
}

void AdafruitIMUI2C::readRegister(unsigned char addr_reg, unsigned char *buffer, unsigned char len) {
    i2c->readReg(addr_reg,(char*)buffer,len);
}

unsigned char AdafruitIMUI2C::readRegister(unsigned char addr_reg) {
    char val;
    i2c->readReg(addr_reg,&val,1);
    return val;
}

void AdafruitIMUI2C::readCalibrationFile(const char * file_path, unsigned char* buffer, int size) {
    std::string str;
    memset(buffer, 0, 22);
    std::ifstream file(file_path);
    getline (file, str);
    file.close();
    int i = 0;
    int idx_buffer = 0;
    std::string temp_str;
    while(str[i] != '\0'){
        temp_str += str[i];
        if(str[i] == ' ')
        {
            buffer[idx_buffer] = (unsigned char)atoi(temp_str.c_str());
            idx_buffer++;
            temp_str = "";
        }
        i++;
    }
    buffer[idx_buffer] = (unsigned char)atoi(temp_str.c_str());
}