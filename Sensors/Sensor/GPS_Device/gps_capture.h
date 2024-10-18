//
// Created by ubuntu on 05/12/22.
//

#ifndef OUTDOORNAV_GPS_CAPTURE_H
#define OUTDOORNAV_GPS_CAPTURE_H

#include "Common/geographic_coordinate.h"
#include "Common/time.h"

struct GPSMotion
{
    float speed;
    float angle;
    float magvariation;
    float HDOP; // Horizontal Dilution of Precision - relative accuracy
    float VDOP; // Vertical Dilution of Precision - relative accurac
    float PDOP; // Position Dilution of Precision - Complex maths derives

    GPSMotion()
    {
        speed = angle = magvariation = HDOP = PDOP = VDOP = 0;
    }
};

struct GPS
{
    Time time;
    GeographicCoordinate coor2D;
    GPSMotion motion;
    unsigned char altitude;
    unsigned char  satellites;    ///< Number of satellites in use
    unsigned char  fixquality;    ///< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
    unsigned char  antenna;       ///< Antenna that is used (from PGTOP)
    GPS()
    {
        time = Time();
        coor2D = GeographicCoordinate();
        satellites = fixquality = antenna = 0;
    }
};

class GPSCapture {
public:
    virtual void start(const char* fdi, int baudrate)=0; // start the capture
    virtual void release() = 0; // release the capture
    virtual char readNextValue() = 0; // Read the next value
    virtual void write(char * cmd, int size) = 0;

    GPS data;
    bool fix = false;
private:
};

#endif //OUTDOORNAV_GPS_CAPTURE_H
