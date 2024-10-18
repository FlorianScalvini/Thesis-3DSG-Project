//
// Created by ubuntu on 26/10/22.
//

#include "adafruit_gps.h"

#include <string>
#include <cstdio>      // standard input / output functions
#include <cstdlib>
#include <cstring>     // string function definitions
#include <cerrno>      // Error number definitions
#include <cmath>
#include <unistd.h>
#include <algorithm>
#include <sstream>
#include "../Board/jetson_orin_AGX.h"
#include "Common/utils.h"
#ifdef JETSON
#include "../Board/jetson_orin_AGX.h"
#endif


double convertDegMinToDeg(std::string degMin);

AdafruitGPS* AdafruitGPS::__singleton = nullptr;

AdafruitGPS* AdafruitGPS::getSingleton()
{
    if (__singleton ==nullptr) {
        __singleton = new AdafruitGPS();
    }
    return __singleton;
}

AdafruitGPS::AdafruitGPS() {
    serial = new UART();
    endTrame = -1;
}


AdafruitGPS::~AdafruitGPS()
{
    if(serial != nullptr)
    {
        serial->closeFDI();
        delete serial;
        serial = nullptr;
    }
}

void AdafruitGPS::write(char * cmd, int size)
{

    serial->writeUART((unsigned char*)cmd, size);
    serial->writeUART((unsigned char*)"\r\n", 2);
    printf("Command sent !\n");
}


void AdafruitGPS::start(const char* fdi, int baudrate){
    this->fdi = fdi;
    serial->openFDI(this->fdi.c_str(), B9600, 1,0);
    fix = false;
    memset(read_buf, 0, 120*sizeof(unsigned char));
    //serial->readUART((unsigned char*)read_buf, sizeof(read_buf)); // Init read;
}



char AdafruitGPS::readNextValue()
{
    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    memset(read_buf, 0, MAXLINELENGTH*sizeof(unsigned char));
    bool frameValidFrame = false;
    while(!frameValidFrame) {
        unsigned int i = 0;
        endTrame = 0;
        while (endTrame == 0) {
            serial->readUART((unsigned char *) &read_buf[i], 1);
            //serial->flushOutput();
            if(read_buf[0] == '$' && i == 0)
                i++;
            else if (read_buf[i] == '$' && i != 0) {
                memset(read_buf, 0, MAXLINELENGTH*sizeof(unsigned char));
                i=0;
            } else if (i != 0 && read_buf[i] == '\n') {
                endTrame = i;
            } else if (i != 0) {
                i++;
            }
        }
        unsigned char checksum = 0;
        for (int k = 1; k < endTrame - 4; ++k) {
            checksum ^= read_buf[k];
        };
        std::string providedCheck = std::string(1, read_buf[endTrame - 3]) + std::string(1, read_buf[endTrame - 2]);
        unsigned int intProCheck;
        std::stringstream ss;
        ss << std::hex << providedCheck; // Interpret the string as hex
        ss >> intProCheck;
        if (checksum == intProCheck && endTrame > 65)
            frameValidFrame=true;
        else
            i=0;
    }


    //printf("%i\n", num_bytes);
    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    //printf("Read %i bytes. Received message: %s\n", endTrame, read_buf);
    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
    // print it to the screen like this!)
    return this->parse_data();
}

char AdafruitGPS::parse_data()
{
    if (strstr(read_buf, "$GNGGA"))
    {
        return this->parse_gga();
    }
    else if (strstr(read_buf, "$GNRMC"))
    {
        return this->parse_rmc();
    }
    else
        return EXIT_FAILURE;
}

char AdafruitGPS::parse_gga()
{
    fix = true;
    char* p = read_buf;
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    data.time.hour = time / 10000;
    data.time.minute = (time % 10000) / 100;
    data.time.seconds = (time % 100);
    data.time.milliseconds = fmod(timef, 1.0) * 1000;


    // parse out latitude
    p = strchr(p, ',')+1;
    std::string strLat;
    while(*p != ',')
    {
        strLat.append(1, *p);
        p++;
    }
    data.coor2D.latitude = convertDegMinToDeg(strLat);
    p++;
    if (p[0] == 'N' || p[0] == 'S') {
        if (p[0] == 'S')
            data.coor2D.latitude = -data.coor2D.latitude;
    }
    else
    {
        fix = false;
        return EXIT_FAILURE;
    }

    // parse out longitude
    p = strchr(p, ',')+1;
    std::string strLong;
    while(*p != ',')
    {
        strLat.append(1, *p);
        p++;
    }
    data.coor2D.longitude = convertDegMinToDeg(strLong);
    p++;
    if (p[0] == 'W' || p[0] == 'E') {
        if (p[0] == 'W')
            data.coor2D.longitude = -data.coor2D.longitude;
    }
    else
    {
        fix = false;
        return EXIT_FAILURE;
    }

    p = strchr(p, ',') + 1;
    data.fixquality = atoi(p);

    p = strchr(p, ',') + 1;
    data.satellites = atoi(p);

    p = strchr(p, ',') + 1;
    data.motion.HDOP = atof(p);

    p = strchr(p, ',') + 1;
    data.altitude = atof(p);
    return EXIT_SUCCESS;
}

char AdafruitGPS::parse_rmc()
{
    char *p = read_buf;
    // get time
    p = strchr(p, ',')+1;
    if(p == nullptr)
        EXIT_FAILURE;
    int timef = (int)atof(p);
    double integer, fraction;
    uint32_t time = timef;
    data.time.hour = time / 10000;
    data.time.minute = (time % 10000) / 100;
    data.time.seconds = (time % 100);
    char milli[3] = {0,0, 0};
    int i = -1;
    while(*p != ',')
    {
        if(*p == '.')
            i = 0;
        else if(i >= 0)
        {
            milli[i] = *p;
            i++;
        }
        p++;
    }
    if(i >= 0)
        data.time.milliseconds = atof(milli);
    else
        data.time.milliseconds = 0;


    //printf("Time: %i:%i:%i::%i\n", data.time.hour, data.time.minute, data.time.seconds, data.time.milliseconds);
    p++;
    // Serial.println(p);
    if (p[0] != 'A'){
        fix = false;
        return EXIT_FAILURE;
    }
    // parse out latitude
    p = strchr(p, ',')+1;
    std::string strLat;
    while(*p != ',')
    {
        strLat.append(1, *p);
        p++;
    }
    data.coor2D.latitude = convertDegMinToDeg(strLat);
    p++;
    if (p[0] == 'N' || p[0] == 'S') {
        if (p[0] == 'S')
            data.coor2D.latitude = -data.coor2D.latitude;
    }
    else
    {
        fix = false;
        return EXIT_FAILURE;
    }

    // parse out longitude
    p = strchr(p, ',')+1;
    std::string strLon;
    while(*p != ',')
    {
        strLon.append(1, *p);
        p++;
    }
    data.coor2D.longitude = convertDegMinToDeg(strLon);
    p++;
    if (p[0] == 'W' || p[0] == 'E') {
        if (p[0] == 'W')
            data.coor2D.longitude = -data.coor2D.longitude;
    }
    else
    {
        fix = false;
        return EXIT_FAILURE;
    }
    // speed
    p = strchr(p, ',')+1;
    data.motion.speed = atof(p);

    // angle
    p = strchr(p, ',')+1;
    data.motion.angle = atof(p);

    p = strchr(p, ',')+1;
    uint32_t fulldate = atof(p);
    data.time.day = fulldate / 10000;
    data.time.month = (fulldate % 10000) / 100;
    data.time.year = (fulldate % 100);

    // we dont parse the remaining, yet!
    return EXIT_SUCCESS;
}


void AdafruitGPS::release() {
    if(serial != nullptr)
    {
        serial->closeFDI();
    }
}

double convertDegMinToDeg(std::string degMin) {
    double decDeg;
    int pos = degMin.find('.');
    std::string deg = degMin.substr(0, pos - 2);
    std::string min = degMin.substr(pos - 2);
    decDeg =  std::stod(deg) + std::stod(min) / 60.0;
    return decDeg;
}
