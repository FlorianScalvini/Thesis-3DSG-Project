//
// Created by ubuntu on 28/10/22.
//

#include "uart.h"
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>

char UART::openFDI(const char* file, tcflag_t baudrate, unsigned char vmin, unsigned char vtime) {
    fdi = open(file, O_RDWR);
    if (fdi < 0) {
        perror("Failed to open the serial port");
        return EXIT_FAILURE;
    }
    struct termios options;
    tcgetattr(fdi, &options);
    options.c_cflag = baudrate | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN]  = vmin;
    options.c_cc[VTIME] = vtime;  // timeout after .1s that isn't working
    tcflush(fdi, TCIOFLUSH);
    tcsetattr(fdi, TCSANOW, &options);
    return EXIT_SUCCESS;
}

long UART::readUART(unsigned char *buf, unsigned int size){
    if (fdi == -1)
        return -1;
    int numRead, i;
    for(i = 0; i < size; i++)
    {
        numRead =read(this->fdi, buf+i, 1);
        if(numRead == -1)
            break;
    }
    return i;
}

long UART::writeUART(unsigned char *buf, unsigned int size){
    if (fdi == -1)
        return -1;
    return write(this->fdi, buf, size);
}


long UART::readUART2(unsigned char *buf, unsigned int size){
    if (fdi == -1)
        return -1;
    int nbRead = ceil(size / 64.0);
    int lstRead = size % 64;
    lstRead = lstRead == 0 ? 64 : lstRead;
    long numRead = 0;
    for(int i = 0; i < nbRead; i++)
    {
        if(i == nbRead - 1)
            numRead += read(this->fdi, (void*)(buf+numRead), lstRead);
        else
            numRead += read(this->fdi, (void*)(buf+numRead), 64);
        if(numRead != 64)
            break;
    }
    this->flushOutput();
    return numRead;
}

char UART::closeFDI() const {
    return (char) close(fdi);
}

void UART::flushInput()
{
    tcflush(fdi, TCIFLUSH);
}

void UART::flushOutput()
{
    tcflush(fdi, TCOFLUSH);
}

/*
char UART::setBaudrate(int baudrate) {
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);
    // Save tty settings, also checking for error
    if (tcsetattr(this->fdi, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }
}*/