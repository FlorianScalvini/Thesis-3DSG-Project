//
// Created by ubuntu on 28/10/22.
//

#ifndef OUTDOORNAV_SPI_H
#define OUTDOORNAV_SPI_H
#include <linux/spi/spidev.h>


struct SPI_Config {
    unsigned char mode;
    unsigned char bits_per_word;
    unsigned int speed;
    unsigned short delay;

};

class SPI {
public:
    SPI(unsigned char mode, unsigned char bits_per_word, unsigned int speed, unsigned short delay);
    SPI();
    ~SPI();
    char openFDI(const char* file);
    char readSPI(unsigned char *bufAddr, int sizeAddr, unsigned char *bufRd, int size);
    char writeSPI(unsigned char* buf, int size);
    char closeFDI();

    bool setBitPerWord(unsigned char p_bit);
    bool setSpeed(unsigned int p_speed);
    bool setMode(unsigned char p_mode);
private:
    int fdi;
    bool initialized;
    unsigned char mode;
    unsigned char bits_per_word;
    unsigned int speed;
    unsigned short delay;
    spi_ioc_transfer xfer[2];
};


#endif //OUTDOORNAV_SPI_H
