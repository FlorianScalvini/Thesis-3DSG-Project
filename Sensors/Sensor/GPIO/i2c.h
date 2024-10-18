//
// Created by ubuntu on 28/10/22.
//

#ifndef OUTDOORNAV_I2C_H
#define OUTDOORNAV_I2C_H


class I2C {
public:
    char openFDI(const char* file, unsigned int addr);
    char readReg(unsigned char addr_reg, char* buf, int size);
    char writeReg(unsigned char addr_reg, char* buf, int size);
    char closeFDI();
private:
    int fdi;
};


#endif //OUTDOORNAV_I2C_H
