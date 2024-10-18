//
// Created by ubuntu on 31/10/22.
//

#ifndef OUTDOORNAV_JETSONORINAGX_H
#define OUTDOORNAV_JETSONORINAGX_H

#include "../GPIO/GPIO.h"
#define I2C_FDI "/dev/i2c-1"
#define UART_FDI "/dev/ttyTHS1"
#define SPI_FDI "dev/spidev0.0"

static GPIO_PIN SPI1_CKL = {481,"PZ.03"};
static GPIO_PIN SPI1_MISO = {482,"PZ.04"};
static GPIO_PIN SPI1_MOSI = {483,"PZ.05"};
static GPIO_PIN SPI1_CS0 = {484,"PZ.06"};
static GPIO_PIN SPI1_CS1 = {485,"PZ.07"};

static GPIO_PIN I2C2_CLK = {335,"PCC.07"};
static GPIO_PIN I2C2_DAT = {336,"PDD.00"};

static GPIO_PIN I2C5_CLK = {432,"PN.07"};
static GPIO_PIN I2C5_DAT = {439,"PN.00"};

static GPIO_PIN GPIO_8 = {325,"PPB.01"};
static GPIO_PIN GPIO_9 = {324,"PPB.00"};
static GPIO_PIN GPIO_17 = {444,"PN.01"};
static GPIO_PIN GPIO_27 = {433,"PP.04"};
static GPIO_PIN GPIO_32 = {446,"PP.06"};
static GPIO_PIN GPIO_35 = {391,"PH.00"};

static GPIO_PIN CAN0_DOUT = {316,"PAA.00"};
static GPIO_PIN CAN0_DIN = {317,"PAA.01"};
static GPIO_PIN CAN1_DOUT = {318,"PAA.04"};
static GPIO_PIN CAN1_DIN = {319,"PAA.06"};

static GPIO_PIN I2S2_CLK = {398,"PH.07"};
static GPIO_PIN I2S_DOUT = {399,"PI.00"};
static GPIO_PIN I2S_DIN = {400,"PI.01"};
static GPIO_PIN I2S_FS = {401,"PI.02"};

#endif //OUTDOORNAV_JETSONORINAGX_H