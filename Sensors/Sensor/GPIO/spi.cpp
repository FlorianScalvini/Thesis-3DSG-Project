//
// Created by ubuntu on 28/10/22.
//

#include "spi.h"
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include "GPIO.h"


SPI::SPI(unsigned char mode, unsigned char bits_per_word, unsigned int speed, unsigned short delay) {
    setMode(mode);
    setBitPerWord(bits_per_word);
    setSpeed(speed);
    this->delay = delay;
}

SPI::~SPI() {
    if(initialized)
        closeFDI();
}

SPI::SPI() {}

char SPI::openFDI(const char* file) {
    if(!initialized)
    {
        /* Open block device */
        fdi = open(file, O_RDWR);
        if (fdi < 0) {
            return fdi;
        }
        initialized = 1;
        return EXIT_SUCCESS;
    }
    else
    {
        printf("The file is already opened");
        return EXIT_FAILURE;
    }
}

char SPI::closeFDI() {
    if(initialized)
        close(fdi);
    return EXIT_SUCCESS;
}

char SPI::readSPI(unsigned char *bufAddr, int sizeAddr, unsigned char *bufRd, int size) {
    xfer[0].rx_buf = (unsigned long)bufAddr;
    xfer[0].len = sizeAddr;
    xfer[0].bits_per_word = bits_per_word;
    xfer[0].speed_hz = speed, //speed
    xfer[1].rx_buf = (unsigned long) bufRd;
    xfer[1].len = size; /* Length of Data to read */
    return ioctl(fdi, SPI_IOC_MESSAGE(2), xfer);
}

char SPI::writeSPI(unsigned char *buf, int size) {
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = size;
    return ioctl(fdi, SPI_IOC_MESSAGE(1), xfer);
}

bool SPI::setBitPerWord(unsigned char p_bit){

    /* Set bits per word*/
    if (ioctl(fdi, SPI_IOC_WR_BITS_PER_WORD, &p_bit) < 0) {
        return false;
    }
    if (ioctl(fdi, SPI_IOC_RD_BITS_PER_WORD, &p_bit) < 0) {
        return false;
    }
    xfer[0].bits_per_word = bits_per_word;
    xfer[1].bits_per_word = bits_per_word;
    return true;

}
bool SPI::setSpeed(unsigned int p_speed){
    /* Set SPI speed*/
    if (ioctl(fdi, SPI_IOC_WR_MAX_SPEED_HZ, &p_speed) < 0) {
        return false;
    }
    if (ioctl(fdi, SPI_IOC_RD_MAX_SPEED_HZ, &p_speed) < 0) {
        return false;
    }
    xfer[0].speed_hz = p_speed;
    xfer[1].speed_hz = p_speed;
    return true;


}

bool SPI::setMode(unsigned char p_mode){
    /* Set SPI_POL and SPI_PHA */
    if (ioctl(fdi, SPI_IOC_WR_MODE, &p_mode) < 0) {
        return false;
    }
    if (ioctl(fdi, SPI_IOC_RD_MODE, &p_mode) < 0) {
        return false;
    }
    mode = p_mode;
    return true;
}