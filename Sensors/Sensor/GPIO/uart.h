//
// Created by ubuntu on 28/10/22.
//

#ifndef OUTDOORNAV_UART_H
#define OUTDOORNAV_UART_H


#include <termios.h> // Contains POSIX terminal control definitions

class UART {
public:
    char openFDI(const char* file, tcflag_t baudrate = B9600, unsigned char vmin=0, unsigned char vtime=0);
    long writeUART(unsigned char* buf, unsigned int size);
    long readUART(unsigned char* buf, unsigned int size) ;
    char closeFDI() const;
    void flushInput();
    void flushOutput();
    long readUART2(unsigned char *buf, unsigned int size);
    //char setBaudrate(int baudrate);
private:
    int fdi;
    //struct termios tty;
};


#endif //OUTDOORNAV_UART_H
