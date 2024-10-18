//
// Created by ubuntu on 24/11/22.
//

#include "adafruit_imu_bno055.h"
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <termios.h>
#include <fstream>
#include <cstdlib>
#include <istream>
#include <cmath>

AdafruitIMU* AdafruitIMU::__singleton = nullptr;

AdafruitIMU* AdafruitIMU::getSingleton()
{
    if (__singleton ==nullptr) {
        __singleton = new AdafruitIMU();
    }
    return __singleton;
}

void AdafruitIMU::start(const char * file) {

    serial = new UART();
    if(serial->openFDI(file, B115200, 0,1))
    {
        std::cout<<"Serial error";
        return;
    }
    this->reset();
    this->setNormalMode();
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00, false);
    this->writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x00, false);
    usleep(10000);
    setMode(modeBNO055::OPERATION_MODE_NDOF);
    usleep(10000);
    std::cout<<"IMU's initialization done"<<std::endl;
}

void AdafruitIMU::release() {
    delete serial;
    serial = nullptr;
}

AdafruitIMU::~AdafruitIMU() {
    this->release();
}

void AdafruitIMU::getQuaternionLinAccGravity(Quaternion &quad, cv::Point3f& lin, cv::Point3f& grv){
    unsigned char buffer[20];
    this->readRegister(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 20);

}

void AdafruitIMU::getGyrAccMag(cv::Point3f& gyr, cv::Point3f& acc, cv::Point3f& mag) {
    unsigned char buffer[18];
    this->readRegister(BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 18);

    acc.x = (float)(((short)(((short) buffer[0]) | (((short) buffer[1]) << 8))) / 100.0);
    acc.y = (float)(((short)(((short) buffer[2]) | (((short) buffer[3]) << 8))) / 100.0);
    acc.z = (float)(((short)(((short) buffer[4]) | (((short) buffer[5]) << 8))) / 100.0);

    mag.x = (float)(((short)(((short) buffer[6]) | (((short) buffer[7]) << 8))) / 16.0);
    mag.y = (float)(((short)(((short) buffer[8]) | (((short) buffer[9]) << 8))) / 16.0);
    mag.z = (float)(((short)(((short) buffer[10]) | (((short) buffer[11]) << 8))) / 16.0);

    acc.x = (float)(((short)(((short) buffer[12]) | (((short) buffer[13]) << 8))) / 0.001090830782496456);
    acc.y = (float)(((short)(((short) buffer[14]) | (((short) buffer[15]) << 8))) / 0.001090830782496456);
    acc.z = (float)(((short)(((short) buffer[16]) | (((short) buffer[17]) << 8))) / 0.001090830782496456);

    printf("%i %i %i %i %i %i -- %i %i %i %i %i %i -- %i %i %i %i %i %i %i %i\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9], buffer[10], buffer[11], buffer[12], buffer[13], buffer[14], buffer[15], buffer[16], buffer[17], buffer[18], buffer[19], buffer[20]);
}

void AdafruitIMU::reset() {
    setMode(modeBNO055::OPERATION_MODE_CONFIG);
    usleep(10000);
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x20);
    usleep(1000000);
}

void AdafruitIMU::setMode(modeBNO055 mode) {
    _mode = mode;
    writeRegister(BNO055_OPR_MODE_ADDR, mode & 0xFF);
    usleep(30000);
}

int AdafruitIMU::getMode() {
    unsigned char val;
    this->readRegister(BNO055_OPR_MODE_ADDR, &val);
    return val;
}


void AdafruitIMU::setNormalMode() {
    writeRegister(BNO055_PWR_MODE_ADDR, 0x00);
    usleep(20000);
}

void AdafruitIMU::setSuspendMode() {
    writeRegister(BNO055_PWR_MODE_ADDR, 0x02);
    /* Set the requested operating mode (see section 3.3) */
    usleep(20000);
}

bool AdafruitIMU::isFullyCalibrated() {
    unsigned char system, gyro, accel, mag;
    this->getCalibration(&system, &gyro, &accel, &mag);
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

void AdafruitIMU::getCalibration(unsigned char *system, unsigned char *gyro, unsigned char *accel, unsigned char *mag) {
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
}


void AdafruitIMU::getDataIMU(DataIMU& output) {
    unsigned char buffer[44];
    this->readRegister(0x08, buffer, 44);
    bufferToPoint3f(&buffer[6],output.mag);
    bufferToPoint3f(&buffer[32],output.linAcc);
    bufferToPoint3f(&buffer[38],output.grav);

    IMUCapture::divide(output.mag, 16.0);
    IMUCapture::divide(output.linAcc, 100.0);
    IMUCapture::divide(output.grav, 100.0);

    const double scale = (1.0 / (1 << 14));
    output.q.q0 = (short)((((unsigned short)buffer[25]) << 8) | ((unsigned short)buffer[24])) * scale;
    output.q.q1 = (short)((((unsigned short)buffer[27]) << 8) | ((unsigned short)buffer[26])) * scale;
    output.q.q2 = (short)((((unsigned short)buffer[29]) << 8) | ((unsigned short)buffer[28])) * scale;
    output.q.q3 = (short)((((unsigned short)buffer[31]) << 8) | ((unsigned short)buffer[30])) * scale;


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
    //float my1 = output.mag.y *

    //printf("Theta %f\n", theta * 180.0 / M_PI);
    output.compass = atan2(output.mag.y * sqrt((1-pow(output.grav.y / 9.8, 2)-pow(output.grav.x / 9.8, 2))) - output.mag.z * output.grav.y / 9.8, output.mag.x *(1-pow(output.grav.x / 9.8, 2)) - output.mag.y*(output.grav.x / 9.8) *(output.grav.y / 9.8)-output.mag.z*(output.grav.x / 9.8)*sqrt((1-pow(output.grav.y / 9.8, 2)-pow(output.grav.x / 9.8, 2))));
    //output.compass= atan2(output.mag.z * sPhi + output.mag.y * cPhi, output.mag.x * cTheta - output.mag.y * sTheta * sPhi + output.mag.z * sTheta * cPhi);
    output.compass = output.compass * 180.0 / M_PI; // Convert to degrees
    output.compass -= convertDegMinToDecDeg(DIJON_MAG_DECLINATION);
    //printf("Compass %f\n", output.compass);
}

/*
 *  Calculate the heading from the magnetometer data and the gravity vector from the accelerometer data.
 */
float AdafruitIMU::getCompass() {
    cv::Point3f pt_mag, pt_grav;
    printf("getCompass");
    this->getMagnetic(pt_mag);
    this->getGravity(pt_grav);
    usleep(10000);

    double phi = atan2(pt_grav.y / 9.8, pt_grav.z / 9.8); // Psi = atan2(gy, gz)
    double theta = atan2(pt_grav.x / 9.8, pt_grav.z / 9.8); // Theta = atan2(-gx, gy * sin(phi) + gz * cos(phi))

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
    double heading = atan2(pt_mag.z * sPhi + pt_mag.y * cPhi, pt_mag.x * cTheta - pt_mag.y * sTheta * sPhi + pt_mag.z * sTheta * cPhi);
    heading = heading * 180.0 / M_PI; // Convert to degrees
    heading -= convertDegMinToDecDeg(DIJON_MAG_DECLINATION);
    return (float)heading;
}


void AdafruitIMU::getAcceleration(cv::Point3f& pt) {
    this->getValue(Register::BNO055_ACCEL_DATA_X_LSB_ADDR, pt);
    /* 1m/s^2 = 100 LSB */
    IMUCapture::divide(pt, 100.0);
}


/*
 * Return the gravity vector
 */
void AdafruitIMU::getGravity(cv::Point3f& pt) {
    this->getValue(Register::BNO055_GRAVITY_DATA_X_LSB_ADDR, pt);
    /* 1m/s^2 = 100 LSB */
    IMUCapture::divide(pt, 100.0);
}
/*
 * Return the Linear Acceleration vector
 */
void AdafruitIMU::getLinearAcceleration(cv::Point3f& pt) {
    this->getValue(Register::BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, pt);
    /* 1m/s^2 = 100 LSB */
    IMUCapture::divide(pt, 100.0);
}


void AdafruitIMU::getMagnetic(cv::Point3f& pt) {
    this->getValue(Register::BNO055_MAG_DATA_X_LSB_ADDR, pt);
    /* 1uT = 16 LSB */
    IMUCapture::divide(pt, 16.0);
}

void AdafruitIMU::getGyroscope(cv::Point3f& pt) {
    this->getValue(Register::BNO055_GYRO_DATA_X_LSB_ADDR, pt);
    /* 1 degrees = 16 LSB  && 1 degrees =  0.0174533 rad*/
    IMUCapture::multiply(pt, 0.001090830782496456);
}

/*
 * Return the orientation in the quaternion form
 */
void AdafruitIMU::getQuaternion(Quaternion &quad) {
    unsigned char buffer[8];
    /* Read vector data (8 bytes) */
    this->readRegister(Register::BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
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

void AdafruitIMU::getEuler(EulerAngle& angle) {
    cv::Point3f pt;
    this->getValue(Register::BNO055_EULER_H_LSB_ADDR, pt);
    /* 1 degree = 16 LSB */
    AdafruitIMU::multiply(pt, 0.001090830782496456);
}

/*
 * Return the temperature in degree
 */
char AdafruitIMU::getTemperature() {
    unsigned char temp;
    this->readRegister(Register::BNO055_TEMP_ADDR, &temp);
    return (char)temp;
}

/*
 *  Return a vector of 3 elements read from the IMU at the given address
 */
char AdafruitIMU::getValue(Register reg, cv::Point3f& pt) {
    unsigned char buffer[6];
    /* Read vector data (6 bytes) */
    this->readRegister(reg, buffer, 6);
    bufferToPoint3f(buffer, pt);

    //printf("Read return : 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X 0x%02.2X\n",buffer[0],  buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
    //printf("Read return : %i %i %i\n", x,  y, z);
    return EXIT_SUCCESS;
}

void AdafruitIMU::bufferToPoint3f(const unsigned char buffer[6], cv::Point3f& pt) {
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
bool AdafruitIMU::getSensorOffsets(unsigned char *buffer) {
    if (isFullyCalibrated()) {
        modeBNO055 lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        this->readRegister(ACCEL_OFFSET_X_LSB_ADDR, buffer, NUM_BNO055_OFFSET_REGISTERS);
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
bool AdafruitIMU::getSensorOffsets(offsetsBNO055 &offsets_type) {
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
void AdafruitIMU::setSensorOffsets(const unsigned char *calibData) {
    modeBNO055 lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    usleep(25000);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    signed char err;
    /* A writeLen() would make this much cleaner */
    this->writeRegister(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    this->writeRegister(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    this->writeRegister(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    this->writeRegister(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    this->writeRegister(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    this->writeRegister(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    this->writeRegister(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
    this->writeRegister(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
    this->writeRegister(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
    this->writeRegister(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
    this->writeRegister(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
    this->writeRegister(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

    this->writeRegister(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
    this->writeRegister(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
    this->writeRegister(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
    this->writeRegister(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
    this->writeRegister(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
    this->writeRegister(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

    this->writeRegister(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
    this->writeRegister(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

    this->writeRegister(MAG_RADIUS_LSB_ADDR, calibData[20]);
    this->writeRegister(MAG_RADIUS_MSB_ADDR, calibData[21]);

    this->setMode(lastMode);
}

void AdafruitIMU::setCalibration(const char *file) {
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
void AdafruitIMU::setSensorOffsets(const offsetsBNO055 &offsets_type) {
    modeBNO055 lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    usleep(25000);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    writeRegister(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    writeRegister(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    this->writeRegister(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
    this->writeRegister(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    this->writeRegister(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
    this->writeRegister(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    this->writeRegister(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
    this->writeRegister(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    this->writeRegister(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
    this->writeRegister(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    this->writeRegister(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
    this->writeRegister(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    this->writeRegister(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
    this->writeRegister(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    this->writeRegister(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
    this->writeRegister(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    this->writeRegister(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
    this->writeRegister(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    this->writeRegister(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
    this->writeRegister(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

    this->writeRegister(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
    this->writeRegister(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

    this->setMode(lastMode);
}

void AdafruitIMU::setAccRange(ACC_G reg = ACCEL_4G) {
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value;
    readRegister(BNO055_ACCEL_CONFIG_ADDR, &value);
    unsigned char masked_value = 0b11111100 & value;
    this->writeRegister(BNO055_ACCEL_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
}

char AdafruitIMU::setAccRate(ACC_RATE reg = ACCEL_62_5HZ) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value;
    readRegister(BNO055_ACCEL_CONFIG_ADDR, &value);
    unsigned char masked_value = 0b11100011 & value;
    this->writeRegister(BNO055_ACCEL_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

char AdafruitIMU::setMagRate(MAG_RATE reg = MAGNET_20HZ) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value;
    readRegister(BNO055_MAG_CONFIG_ADDR, &value);
    unsigned char masked_value = 0b01111000 & value;
    this->writeRegister(BNO055_ACCEL_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

char AdafruitIMU::setGyroRange(GYRO_DPS reg = GYRO_2000_DPS) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value;
    readRegister(BNO055_GYRO_0_CONFIG_ADDR, &value);
    unsigned char masked_value = 0b00111000 & value;
    this->writeRegister(BNO055_GYRO_0_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
    return EXIT_SUCCESS;
}

char AdafruitIMU::setGyroRate(GYRO_RATE reg = GYRO_32HZ) {
    if(_mode >= 0x08 && _mode <= 0x0C)
    {
        fprintf(stderr, "Error, impossible to set the gyroscope rate with the current mode\n");
        return EXIT_FAILURE; // Mode must not be a fusion mode
    }
    writeRegister(BNO055_PAGE_ID_ADDR, 0x01);
    unsigned char value;
    readRegister(BNO055_GYRO_0_CONFIG_ADDR, &value);
    unsigned char masked_value = 0b00000111 & value;
    this->writeRegister(BNO055_GYRO_0_CONFIG_ADDR, masked_value | reg);
    this->writeRegister(BNO055_PAGE_ID_ADDR, 0x00);
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}

void AdafruitIMU::getSystemStatus(unsigned char * system_status, unsigned char *self_test_result, unsigned char *system_error) {
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



void AdafruitIMU::writeRegister(unsigned char addr_reg, unsigned char value, bool ack) {
    unsigned char buffer[5] = {0xAA, 0x00, addr_reg, 1,value};
    unsigned char rtnVal[2] = {0, 0};
    serial->writeUART(buffer, 5);
    usleep(2000);
    serial->readUART(rtnVal, 2);
    while(ack && rtnVal[0] != 0xEE && rtnVal[1] != 0x01)
    {
        fprintf(stderr, "Error writing register 0x%02.2X\n", addr_reg);
        serial->writeUART(buffer, 5);
        usleep(1000);
        serial->readUART(rtnVal, 2);
    }
}

void AdafruitIMU::readRegister(unsigned char addr_reg, unsigned char *buffer, unsigned char len) {
    unsigned char recv[2];
    unsigned char buf[4];
    buf[0] = 0xAA;
    buf[1] = 0x01;
    buf[2] = addr_reg & 0xFF;
    buf[3] = len & 0xFF;
    bool writeDone = false;
    long read_len;
    while(!writeDone)
    {
        serial->flushInput();
        serial->writeUART(buf, 4);
        read_len = serial->readUART(recv, 2);
        if(read_len == 2 && recv[0] == 0xBB && recv[1] == len)
            writeDone = true;
        usleep(500);
    }
    read_len = serial->readUART(buffer, len);
    serial->flushOutput();
}

unsigned char AdafruitIMU::readRegister(unsigned char addr_reg) {
    unsigned char buf[4] = {0xAA, 0x01, addr_reg, 1};
    serial->writeUART(buf, 4);
    long read_len = serial->readUART(buf, 4);
    if (read_len != 1 || buf[0] != 0xBB)
    {
        //fprintf(stderr, "Error reading register 0x%02.2X\n", addr_reg);
        return 0x00;
    }
    return buf[2];
}

void AdafruitIMU::readCalibrationFile(const char * file_path, unsigned char* buffer, int size) {
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