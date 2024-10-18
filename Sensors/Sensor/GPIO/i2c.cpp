//
// Created by ubuntu on 28/10/22.
//

#include "i2c.h"
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <cstdint>
#include <cstring>


char I2C::writeReg(unsigned char addr_reg, char* buf, int size) {
    unsigned char outbuf[size+1];
    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    outbuf[0] = addr_reg;
    memcpy(outbuf+1, buf, size);

    msgs[0].addr = 0x28;
    msgs[0].flags = 0;
    msgs[0].len = size+1;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(fdi, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_write");
        return -1;
    }
    return 0;
}

char I2C::readReg(unsigned char addr_reg, char *buf, int size) {
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = 0x28;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &addr_reg;

    msgs[1].addr = 0x28;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = size;
    msgs[1].buf = (unsigned char*)buf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    buf[0] = 0;
    if (ioctl(fdi, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return -1;
    }
    return 0; //Lecture du registre
}

char I2C::openFDI(const char* file, unsigned int addr) {
    struct i2c_smbus_ioctl_data args;
    fdi = open(file, O_RDWR);
    if (fdi < 0) {
        return -1;
    }

    if (ioctl(fdi, I2C_SLAVE, addr) < 0) {
        perror("Failed to set I2C device address");
        return -1;
    }

    return 0;
}

char I2C::closeFDI() {
    return close(this->fdi);
}

