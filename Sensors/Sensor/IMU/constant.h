//
// Created by ubuntu on 04/11/22.
//

#ifndef OUTDOORNAV_CONSTANT_H
#define OUTDOORNAV_CONSTANT_H

#define JETSON

/*
    IMU Structure
*/


struct Point3d {
    double x; // DPS in X direction
    double y; // DPS in Y direction
    double z; // DPS in Z direction
    Point3d(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    Point3d()
    {
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }
};



struct PolarData {
    float r; // the total magnetic field computed from the three axes components;
    float d; // the angle between Z axis and the rezultant of X and Y
    PolarData(float R, float D)
    {
        this->r = R;
        this->d = D;
    }
    PolarData()
    {
        this->r = 0;
        this->d = 0;
    }
};


struct SensorID {
    unsigned char ag; // device ID for AG instrument
    unsigned char mag; // device ID for MAG instrument
    unsigned char alt; // device ID for ALT instrument
    SensorID()
    {
        this->ag = 0;
        this->mag = 0;
        this->alt = 0;
    }
};

#endif //OUTDOORNAV_CONSTANT_H
