//
// Created by ubuntu on 27/10/22.
//

#include "GPIO.h"
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

char GPIO::exportGpioPin(GPIO_PIN gpio) {
    char buf[5];
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        printf("Error, the GPIO port can't be export. \n");
        return EXIT_FAILURE;
    }
    sprintf(buf, "%i%c", gpio, '\0');
    write(fd, buf, strlen(buf));
    close(fd);
    usleep(10000);
    printf("GPIO %i exported \n", gpio.gpio_id);
    return EXIT_SUCCESS;
}


char GPIO::direction(GPIO_PIN gpio, char direction) {
    char buf[60];
    sprintf(buf,"/sys/class/gpio/%s/direction%c",gpio.gpio_path,'\0');
    int fd = open(buf, O_WRONLY);
    if (fd < 0) {
        printf("Error setting direction, the GPIO port %i can't be open. \n", gpio.gpio_id);
        return EXIT_FAILURE;
    }
    if(direction == INPUT)
        sprintf(buf, "%in%c", direction,'\0');
    else if(direction == OUTPUT)
        sprintf(buf, "%out%c", direction,'\0');
    else
        return EXIT_FAILURE;
    write(fd, buf, strlen(buf));
    close(fd);
    return EXIT_SUCCESS;
}


char GPIO::readGPIOValue(GPIO_PIN gpio, char &val) {
    char buf[50];
    sprintf(buf,"/sys/class/gpio/%s/value",gpio.gpio_path);
    int fd = open(buf, O_RDONLY);

    if (fd < 0) {
        printf("Error reading, the GPIO port %i can't be open. \n"), gpio;
    }
    int num_bytes = read(fd, buf, 1);
    if(num_bytes != 1)
    {
        printf("Error reading value at the GPIO port %i.\n", gpio.gpio_id);
        return EXIT_FAILURE;
    }
    close(fd);
    val = buf[0];
    return EXIT_SUCCESS;
}


char GPIO::writeGPIOValue(GPIO_PIN gpio, char value) {
    if(value == LOW || value == HIGH){
        char buf[50];
        sprintf(buf,"/sys/class/gpio/%s/value",gpio.gpio_path);
        int fd = open(buf, O_WRONLY);
        if (fd < 0) {
            printf("Error writing, the GPIO port %i can't be open.\n", gpio.gpio_id);
        }
        sprintf(buf, "%c%c", value,'\0');
        write(fd, buf, strlen(buf));
        close(fd);
        return EXIT_SUCCESS;
    }
    else
    {
        printf("Error, the value is incorrect. \n");
        return EXIT_FAILURE;
    }
}


char GPIO::unExportPin(GPIO_PIN gpio) {
    char buf[5];
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        printf("Error, the GPIO port %i  can't be unexport. \n", gpio);
        return EXIT_FAILURE;
    }
    sprintf(buf, "%d", gpio);
    write(fd, buf, strlen(buf));
    close(fd);
    return EXIT_SUCCESS;
}