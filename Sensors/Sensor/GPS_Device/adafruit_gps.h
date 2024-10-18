//
// Created by ubuntu on 26/10/22.
//

#ifndef OUTDOORNAV_ADAFRUIT_GPS_H
#define OUTDOORNAV_ADAFRUIT_GPS_H

#include "gps_capture.h"
#include "gps_constant.h"
#include <string>
#include "../GPIO/uart.h"


class AdafruitGPS : public GPSCapture {
public:
    AdafruitGPS();
    void start(const char* fdi, int baudrate=9600) override;
    void release() override;
    ~AdafruitGPS();
    void write(char *, int);
    char readNextValue() override;
    char parse_data();
    static AdafruitGPS* __singleton;
    static AdafruitGPS* getSingleton();

private:
    char parse_rmc();
    char parse_gga();
    char parse_ggl() {return false;}; // Unimplemented
    char parse_gsa() {return false;}; // Unimplemented

    //Attribut
    char read_buf [MAXLINELENGTH];
    unsigned int endTrame;
    std::string fdi;
    UART* serial;
};


#endif //OUTDOORNAV_ADAFRUIT_GPS_H