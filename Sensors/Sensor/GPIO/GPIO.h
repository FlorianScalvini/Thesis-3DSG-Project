//
// Created by ubuntu on 27/10/22.
//

#ifndef OUTDOORNAV_GPIO_H
#define OUTDOORNAV_GPIO_H

#define INPUT 0
#define OUTPUT 1

#define HIGH '1'
#define LOW '0'


struct GPIO_PIN
{
    int gpio_id;
    const char * gpio_path;
};

static bool operator==(const GPIO_PIN& a, const GPIO_PIN& b)
{
    return a.gpio_id == b.gpio_id;
}

static bool operator!=(const GPIO_PIN& a, const GPIO_PIN& b)
{
    return a.gpio_id != b.gpio_id;
}

static bool operator!=(const GPIO_PIN& a, int b)
{
    return a.gpio_id != b;
}

static bool operator==(const GPIO_PIN& a, int b)
{
    return a.gpio_id == b;
}

class GPIO {
public:
    static char exportGpioPin(GPIO_PIN gpio);

    static char direction(GPIO_PIN gpio, char direction);
    static char writeGPIOValue(GPIO_PIN gpio, char value);
    static char readGPIOValue(GPIO_PIN gpio, char &value);

    static char unExportPin(GPIO_PIN gpio);
};

#endif //OUTDOORNAV_GPIO_H