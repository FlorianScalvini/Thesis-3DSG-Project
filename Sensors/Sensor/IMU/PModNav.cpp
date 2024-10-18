//
// Created by ubuntu on 27/10/22.
//

#include "PModNav.h"


#include "../Board/jetson_orin_AGX.h"

#define CS_AG GPIO_8
#define CS_M GPIO_17
#define CS_ALT GPIO_27
#define DRDY_M GPIO_32
#define INT_GPIO GPIO_35


PModNav::PModNav() {
    this->m_GRangeLSB = 0;
    this->m_DPSRangeLSB = 0;
    this->m_GaussRangeLSB = 0;
    this->Pref = 0;
    this->tempC = 0;
    this->hPa = 0;
    this->id = SensorID();
    this->begin();
}

PModNav::~PModNav() {
    this->end();
}

void PModNav::begin() {
    m_GRangeLSB = this->getXLRangeLSB(PAR_XL_2G);	// the startup range for the accelerometer is +/- 2g, which corresponds to a LSB value of 0.061mg/LSB
    m_DPSRangeLSB = this->getGRangeLSB(PAR_G_245DPS);	// the startup range for the gyro is +/- 245dps, which corresponds to a LSB value of 8.75mdps/LSB
    m_GaussRangeLSB = this->getMAGRangeLSB(PAR_MAG_4GAUSS);// the startup range for the magnetometer is +/- 4Gauss, which corresponds to a LSB value of 0.14mGauss/LSB
    Pref = 1013.25; //reference pressure corresponding to sea level
    this->navHostInit();
    this->navDevInit();
    this->getDeviceID();
}

void PModNav::end() {
    navDevTerm();
    navHostTerm();
    m_GRangeLSB = 0;
    m_DPSRangeLSB = 0;
    m_GaussRangeLSB = 0;
}

void PModNav::getDeviceID() {
    readRegister(CS_AG, WHO_AM_I, &id.ag, 1);
    readRegister(CS_M, WHO_AM_I_M, &id.mag, 1);
    readRegister(CS_ALT, WHO_AM_I, &id.alt, 1);
}

/* ------------------------------------------------------------ */
/*	PModNav::PModNavHostInit()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Performs SPI interface initialization to prepare for use
**		of the PModNav module. Sets the module pins as output or input, according to their functionalities

*/
void PModNav::navHostInit()
{
    spi.openFDI(SPI_FDI);
    spi.setMode(3);
    spi.setSpeed(4000000);

    //output pins for accelerometer, magnetometer and altimeter SS
    GPIO::exportGpioPin(CS_AG);
    GPIO::direction(CS_AG, OUTPUT);
    GPIO::writeGPIOValue(CS_AG, HIGH);

    GPIO::exportGpioPin(CS_M);
    GPIO::direction(CS_M, OUTPUT);
    GPIO::writeGPIOValue(CS_M, HIGH);

    GPIO::exportGpioPin(CS_ALT);
    GPIO::direction(CS_ALT, OUTPUT);
    GPIO::writeGPIOValue(CS_ALT, HIGH);

    if(DRDY_M != -1)
    {
        GPIO::exportGpioPin(DRDY_M);
        GPIO::direction(DRDY_M, INPUT);
        GPIO::writeGPIOValue(DRDY_M, LOW);
    }
    if(INT_GPIO != -1)
    {
        GPIO::exportGpioPin(INT_GPIO);
        GPIO::direction(INT_GPIO, INPUT);
        GPIO::writeGPIOValue(INT_GPIO, LOW);
    }
}

/* ------------------------------------------------------------ */
/*	PModNav::navHostTerm()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Releases processor resources used by the library, defaults processor's pins
*/

void PModNav::navHostTerm()
{
    // Make the signal pins be inputs.
    GPIO::direction(CS_AG, INPUT);
    GPIO::writeGPIOValue(CS_AG, HIGH);
    GPIO::direction(CS_M, INPUT);
    GPIO::writeGPIOValue(CS_M, HIGH);
    GPIO::direction(CS_ALT, INPUT);
    GPIO::writeGPIOValue(CS_ALT, HIGH);
    if(DRDY_M != -1)
    {
        GPIO::direction(DRDY_M, INPUT);
        GPIO::writeGPIOValue(DRDY_M, LOW);
        GPIO::unExportPin(DRDY_M);
    }
    if(INT_GPIO != -1)
    {
        GPIO::direction(INT_GPIO, INPUT);
        GPIO::writeGPIOValue(INT_GPIO, LOW);
        GPIO::unExportPin(DRDY_M);
    }
    GPIO::unExportPin(CS_M);
    GPIO::unExportPin(CS_ALT);
    GPIO::unExportPin(CS_ALT);
}
/* ------------------------------------------------------------ */
/*	PModNav::navDevInit()
**
**	Parameters:
**
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Any initialization needed.
**
**
*/
void PModNav::navDevInit()
{
    this->init();
}
/* ------------------------------------------------------------ */
/*	PModNav::navDevTerm()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Shuts down the PModNav hardware
*/
void PModNav::navDevTerm()
{
    //shuts down the accel and gyro instruments
    initAG(0, MODE_INST_AG);
    //shuts down the magnetometer instrument
    initMAG(0);
    //shuts down the altimeter instrument
    initALT(0);
}

void PModNav::init() {
    this->initAG(1, MODE_INST_AG);
    this->initALT(1);
    this->initALT(1);
}

void PModNav::initAG(bool fInit, unsigned char bModeSel) {
    if (fInit)
    {
        if (bModeSel==MODE_INST_A)
        {
            //enable all three axes
            writeSPI(CS_AG, CTRL_REG5_XL,0x38);
            //set 10Hz odr for accelerometer
            writeSPI(CS_AG, CTRL_REG6_XL, 0x20);
        }
        else if(bModeSel==MODE_INST_AG)
        {
            //enable all three axes
            writeSPI(CS_AG, CTRL_REG5_XL,0x38);
            //set 10Hz odr for accel when used together with gyro
            writeSPI(CS_AG, CTRL_REG6_XL, 0x20);
            //set 10Hz rate for Gyro
            writeSPI(CS_AG, CTRL_REG1_G,0x20);
            //enable the axes outputs for Gyro
            writeSPI(CS_AG, CTRL_REG4,0x38);
        }
    }
    else
    {
        if (bModeSel==MODE_INST_A)
        {
            //power down accel
            writeSPI(CS_AG, CTRL_REG5_XL, 0x00);
            writeSPI(CS_AG, CTRL_REG6_XL, 0x00);
        }
        else if(bModeSel==MODE_INST_AG)
        {
            //power down both the accel and gyro instruments
            writeSPI(CS_AG, CTRL_REG5_XL, 0x00);
            writeSPI(CS_AG, CTRL_REG6_XL, 0x00);
            writeSPI(CS_AG, CTRL_REG9, 0x00);

            writeSPI(CS_AG, CTRL_REG4, 0x00);
            writeSPI(CS_AG, CTRL_REG1_G, 0x00);
        }
    }
}

void PModNav::initMAG(bool fInit) {
    if (fInit)
    {
        //set medium performance mode for x and y and 10Hz ODR for MAG,
        writeSPI(CS_M, CTRL_REG1_M, 0x30);
        //set scale to +-4Gauss
        writeSPI(CS_M, CTRL_REG2_M, 0);
        //disable I2C and enable SPI read and write operations,
        //set the operating mode to continuous conversion
        writeSPI(CS_M, CTRL_REG3_M, 0x00);
        //set medium performance mode for z axis
        writeSPI(CS_M, CTRL_REG4_M, 0x04);
        //cntinuous update of output registers
        writeSPI(CS_M, CTRL_REG5_M, 0x00);
    }
    else
    {
        //power down the instrument
        writeSPI(CS_M, CTRL_REG3_M,0x03);
    }
}

void PModNav::initALT(bool fInit) {
    if (fInit)
    {
        //clean start
        writeSPI(CS_ALT, CTRL_REG1, 0x00);
        usleep(1000);
        //set active the device and ODR to 7Hz
        writeSPI(CS_ALT, CTRL_REG1, 0xA4);
        //increment address during multiple byte access disabled
        writeSPI(CS_ALT, CTRL_REG2, 0x00);
        //no modification to interrupt sources
        writeSPI(CS_ALT, CTRL_REG4, 0x00);
    }
    else
    {
        //power down the instrument
        writeSPI(CS_ALT, CTRL_REG1,0x00);
    }
}


/*-------------------------------------------------------------*/
/*	Accelerometer Specific Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*   PModNav::readAccel(short &AclX, short &AclY, short &AclZ)
**
**  Parameters:
**		&AclX	- the output parameter that will receive acceleration on X axis - 16 bits value
**		&AclY	- the output parameter that will receive acceleration on Y axis - 16 bits value
**		&AclZ	- the output parameter that will receive acceleration on Z axis - 16 bits value
**
**  Return Values:
**      none
**
**  Errors:
**		none
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the accelerometer.
**			-	It reads simultaneously the acceleration on three axes in a buffer of 6 bytes using the this->readRegister function
**			-	For each of the three axes, it combines the two bytes in order to get a 16-bit value
*/
void PModNav::readAccel(short &AclX, short &AclY, short &AclZ)
{
    unsigned char iAclX_L, iAclX_H, iAclY_L, iAclY_H, iAclZ_L, iAclZ_H;
    unsigned char rgwRegVals[6];
    //reads the bytes using the incremeting address functionality of the device.
    this->readRegister(CS_AG, OUT_X_L_XL, (unsigned char *)rgwRegVals, 6);
    iAclX_L = rgwRegVals[0];
    iAclX_H = rgwRegVals[1];
    iAclY_L = rgwRegVals[2];
    iAclY_H = rgwRegVals[3];
    iAclZ_L = rgwRegVals[4];
    iAclZ_H = rgwRegVals[5];
    //combines the read values for each axis to obtain the 16-bit values
    AclX = ((short)iAclX_H << 8) | iAclX_L;
    AclY = ((short)iAclY_H << 8) | iAclY_L;
    AclZ = ((short)iAclZ_H << 8) | iAclZ_L;
}
/* ------------------------------------------------------------ */
/*  PModNav::readAccelG(float &AclXg, float &AclYg, float &AclZg)
**
**  Parameters:
**		&AclXg	- the output parameter that will receive acceleration on X axis (in "g")
**		&AclYg	- the output parameter that will receive acceleration on Y axis (in "g")
**		&AclZg	- the output parameter that will receive acceleration on Z axis (in "g")
**
**  Return Values:
**      none
**
**  Errors:
**		none
**  Description:
**		This function is the main function used for accelerometer values reading, providing the 3 current accelerometer values in �g�.
**		It returns the acceleration measured on the three axes in "g", using the raw values and the conversion function
**		For each of the three values, converts the 16-bit value to the value expressed in �g�, considering the currently selected g range
*/
void PModNav::readAccelG(float &AclXg, float &AclYg, float &AclZg)
{
    short AclX, AclY, AclZ;

    this->readAccel(AclX, AclY, AclZ);
    AclXg = this->convertReadingToValueG(AclX);
    AclYg = this->convertReadingToValueG(AclY);
    AclZg = this->convertReadingToValueG(AclZ);
}
/* ------------------------------------------------------------ */
/*   PModNav::convertReadingToValueG(short rawVal)
**
**  Parameters:
**		rawVal	- the 2 bytes containing the reading.
**
**  Return Values:
**      float - the value of the acceleration in "g" corresponding to the 16 bits reading and the current g range
**
**  Errors:
**		none
**  Description:
**		Converts the value from the 16 bits reading to the float value (in g) corresponding to the acceleration, considering the current selected g range.
*/
float PModNav::convertReadingToValueG(short rawVal)
{
    //Convert the accelerometer value to G's.
    float dResult = ((float)rawVal) * m_GRangeLSB;
    return dResult;
}
/* ------------------------------------------------------------ */
/*  PModNav::GetXLRangeLSB(unsigned char bRangeXL)
**
**  Parameters:
**		bRangeXL	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_XL_2G	Parameter g range : +/- 2g
**					1	PAR_XL_4G	Parameter g range : +/- 4g
**					2	PAR_XL_8G	Parameter g range : +/- 8g
**					3	PAR_XL_16G 	Parameter g range : +/- 16g
**
**
**  Return Value:
**     float - the corresponding value of one LSB unit according to the range set
**
**  Errors:
**	   none
**
**  Description:
**		The function computes the range LSB based on the set range parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "g" value.
**		If value is outside this range, the default value is set.
*/
float PModNav::getXLRangeLSB(unsigned char bRangeXL)
{
    float gRangeLSB;
    switch(bRangeXL)
    {
        case PAR_XL_2G:
            gRangeLSB = 0.000061;
            break;
        case PAR_XL_4G:
            gRangeLSB = 0.000122;
            break;
        case PAR_XL_8G:
            gRangeLSB = 0.000244;
            break;
        case PAR_XL_16G:
            gRangeLSB = 0.000732;
            break;
        default:
            gRangeLSB = 0.000061;
            break;
    }
    return gRangeLSB;
}
/* ------------------------------------------------------------ */
/*   PModNav::setRangeXL(unsigned char bRangeXL)
**
**  Parameters:
**		bRangeXL	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_XL_2G	Parameter g range : +/- 2g
**					1	PAR_XL_4G	Parameter g range : +/- 4g
**					2	PAR_XL_8G	Parameter g range : +/- 8g
**					3	PAR_XL_16G  Parameter g range : +/- 16g
**
**
**  Return Value:
**      none
**
**  Errors:
**		none
**  Description:
**		The function sets the appropriate g range bits in the CTRL_REG6_XL register.
**
*/
void PModNav::setRangeXL(unsigned char bRangeXL)
{
    m_GRangeLSB = this->getXLRangeLSB(bRangeXL);
    this->setBitsInRegister(CS_AG, CTRL_REG6_XL, MSK_RANGE_XL, bRangeXL, 3);
}
/* ------------------------------------------------------------ */
/*   PModNav::getRangeXL()
**
**  Parameters:
**		none
**
**  Return Value:
**      unsigned char - returns the previously selected range from CTRL_REG6_XL register
**
**  Errors:
**		none
**  Description:
**		The function reads the g range bits in the CTRL_REG6_XL register and computes the range to be provided to the user.
**		The accepted argument values are between 0 and 3. If the value is not a valid one, the default value for range is set
**
*/
unsigned char PModNav::getRangeXL()
{
    unsigned char readRange, gRange;
    readRange = this->getBitsInRegister(CS_AG, CTRL_REG6_XL, 3, 2);
    switch(readRange)
    {
        case PAR_XL_2G:
            gRange = 2;
            break;
        case PAR_XL_4G:
            gRange = 4;
            break;
        case PAR_XL_8G:
            gRange = 8;
            break;
        case PAR_XL_16G:
            gRange = 16;
            break;
        default:
            gRange = 2;
            break;
    }
    return gRange;
}
/* ------------------------------------------------------------ */
/*   PModNav::dataAvailableXL()
**
**  Parameters:
**		none
**
**  Return Value:
**      unsigned char - returns the data available status in STATUS_REG register, for accel
**
**  Errors:
**		none
**
**  Description:
**		The function reads the STATUS_REG register and returns the data available bit in it
**
*/
unsigned char PModNav::dataAvailableXL()
{
    unsigned char status;
    this->readRegister(CS_AG, STATUS_REG, &status, 1);
    return (status & (1<<0));
}
/* ------------------------------------------------------------ */
/*   PModNav::configIntXL(unsigned char bIntGen, bool aoi, bool latch)
**
**  Parameters:
**		bIntGen	- The parameter indicating the interrupt generator. Can be one of the parameters from the following list
**			MSK_XLIE_XL 				1<<0
**			MSK_XHIE_XL 				1<<1
**			MSK_YLIE_XL 				1<<2
**			MSK_YHIE_XL 				1<<3
**			MSK_ZLIE_XL 				1<<4
**			MSK_ZHIE_XL 				1<<5
**			MSK_GEN_6D 					1<<6
**		aoi - parameter indicating whether the interrupt generators are or-ed or and-ed together
**		latch - parameter that sets or not the latch interrupt request
**  Return Value:
**      none
**
**  Errors:
**
**  Description:
**		The function configures the interrupt register INT_GEN_CFG_XL setting the interrupt generator.
**		Sets the interrupt events to or-ed or and-ed. Sets the interrupt event to be latched or not.
*/
void PModNav::configIntXL(unsigned char bIntGen, bool aoi, bool latch)
{
    unsigned char temp = bIntGen;
    //interrupt events are or-ed or and-ed
    if (aoi)
    {
        temp |= 0x80;
    }
    this->writeSPI(CS_AG, INT_GEN_CFG_XL, temp);
    temp = 0;

    temp = this->readSPI(CS_AG, CTRL_REG4);
    //latched interrupt enable is set in CTRL_REG4 register
    if (latch)
    {
        temp |= 0x02;
    }
    this->writeSPI(CS_AG, CTRL_REG4, temp);
}
/* ------------------------------------------------------------ */
/*  PModNav::setIntThresholdXL(float thValX, float thValY, float thValZ, unsigned char intDuration, bool wait)
**
**  Parameters:
**		thValX, thValY, thValZ - Parameters containing the threshold value on each axis
**		intDuration - parameter indicating the duration of the enter/exit interrupt
**		wait - parameter enabling or disabling the wait time before exiting the interrupt routine.
**  Return Value:
**      none
**
**  Errors:
**		none
**  Description:
**		The function sets the interrupt threshold for each axis and also the duration of the enter/exit interrupt. Enables or disables the
**		wait on duration before exiting interrupt.
**
*/
void PModNav::setIntThresholdXL(float thValX, float thValY, float thValZ, unsigned char intDuration, bool wait)
{
    unsigned char bthValX, bthValY, bthValZ;
    //converts the float value in g to raw accel data to be written in INT_GEN_THS_X/Y/Z_XL registers
    bthValX = (unsigned char)(thValX/m_GRangeLSB);
    bthValY = (unsigned char)(thValY/m_GRangeLSB);
    bthValZ = (unsigned char)(thValZ/m_GRangeLSB);

    this->writeSPI(CS_AG, INT_GEN_THS_X_XL, bthValX);
    this->writeSPI(CS_AG, INT_GEN_THS_Y_XL, bthValY);
    this->writeSPI(CS_AG, INT_GEN_THS_Z_XL, bthValZ);

    // Write duration and wait to INT_GEN_DUR_XL register
    unsigned char temp;
    temp = (intDuration & 0x7F);
    if (wait) temp |= 0x80;
    this->writeSPI(CS_AG, INT_GEN_DUR_XL, temp);
}


/*-------------------------------------------------------------*/
/*	Gyroscope Specific Functions
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*  PModNav::readGyro(short &GX, short &GY, short &GZ)
**
**  Parameters:
**		&GX	- the output parameter that will receive gyro value on X axis - 16 bits value
**		&GY	- the output parameter that will receive gyro value on Y axis - 16 bits value
**		&GZ	- the output parameter that will receive gyro value on Z axis - 16 bits value
**
**  Return Values:
**      none
**
**  Errors:
**		none
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the gyro.
**			-	It reads simultaneously the gyro value on three axes in a buffer of 6 bytes using the this->readRegister function
**			-	For each of the three axes, combines the two bytes in order to get a 16-bit value
*/
void PModNav::readGyro(short &GX, short &GY, short &GZ)
{
    unsigned char iGX_L, iGX_H, iGY_L, iGY_H, iGZ_L, iGZ_H;
    unsigned char rgwRegVals[6];
    //reads the bytes using the incremeting address functionality of the device.
    this->readRegister(CS_AG, OUT_X_L_G, (unsigned char *)rgwRegVals, 6);
    iGX_L = rgwRegVals[0];
    iGX_H = rgwRegVals[1];
    iGY_L = rgwRegVals[2];
    iGY_H = rgwRegVals[3];
    iGZ_L = rgwRegVals[4];
    iGZ_H = rgwRegVals[5];
    //combines the read values for each axis to obtain the 16-bits values
    GX = ((short)iGX_H << 8) | iGX_L;
    GY = ((short)iGY_H << 8) | iGY_L;
    GZ = ((short)iGZ_H << 8) | iGZ_L;
}

/* ------------------------------------------------------------ */
/*  PModNav::readGyroDps(float &GXdps, float &GYdps, float &GZdps)
**
**  Parameters:
**		&GXdps	- the output parameter that will receive gyro values on X axis (in "dps")
**		&GYdps	- the output parameter that will receive gyro values on Y axis (in "dps")
**		&GZdps	- the output parameter that will receive gyro values on Z axis (in "dps")
**
**  Return Values:
**      none
**
**  Errors:
**		none
**  Description:
**		This function is the main function used for gyro values reading, providing the 3 current gyro values in �dps�.
**		It returns the gyro values measured on the three axes in "dps", using the raw values and the conversion function
**		For each of the three values, converts the 16-bit value to the value expressed in �dps�, considering the currently selected dps range
*/
void PModNav::readGyroDps(float &GXdps, float &GYdps, float &GZdps)
{
    short GX, GY, GZ;

    this->readGyro(GX, GY, GZ);
    GXdps = this->convertReadingToValueDPS(GX);
    GYdps = this->convertReadingToValueDPS(GY);
    GZdps = this->convertReadingToValueDPS(GZ);
}

/* ------------------------------------------------------------ */
/*  PModNav::convertReadingToValueDPS(short rawVal)
**
**  Parameters:
**		rawVal	- the 2 bytes containing the reading.
**
**  Return Values:
**      float - the value of the gyro in "dps" corresponding to the 16 bits reading and the current dps range
**
**  Errors:
**		none
**  Description:
**		Converts the value from the 16 bits reading to the float value (in dps) corresponding to the degrees value, considering the current selected dps range.
**
*/
float PModNav::convertReadingToValueDPS(short rawVal)
{
    //Convert the gyro value to dps.
    float dResult = ((float)rawVal) * m_DPSRangeLSB;
    return dResult;
}

/* ------------------------------------------------------------ */
/*  PModNav::getGRangeLSB
**
**   Parameters:
**		bRangeG	- the parameter specifying the dps range. Can be one of the parameters from the following list:
**					0	PAR_G_245DPS	Parameter dps range : +/- 245dps
**					1	PAR_G_500DPS	Parameter dps range : +/- 500dps
**					3	PAR_G_2kDPS		Parameter dps range : +/- 2kdps
**
**   Return Value:
**      float - corresponding value of one LSB unit according to the range set
**
**   Errors:
**		none
**   Description:
**		The function computes the range LSB based on the set range parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "dps" value.
**		If value is outside this range, the default value is set.
**
*/
float PModNav::getGRangeLSB(unsigned char bRangeG)
{
    float gRangeLSB;
    switch(bRangeG)
    {
        case PAR_G_245DPS:
            gRangeLSB = 0.00875;
            break;
        case PAR_G_500DPS:
            gRangeLSB = 0.0175;
            break;
        case PAR_G_2kDPS:
            gRangeLSB = 0.07;
            break;
        default:
            gRangeLSB = 0.00875;
            break;
    }
    return gRangeLSB;
}

/* ------------------------------------------------------------ */
/*  PModNav::setRangeG(unsigned char bRangeG)
**
**  Parameters:
**		bGRangeG	- the parameter specifying the dps range. Can be one of the parameters from the following list:
**					0	PAR_G_245DPS	Parameter dps range : +/- 245dps
**					1	PAR_G_500DPS	Parameter dps range : +/- 500dps
**					3	PAR_G_2000DPS	Parameter dps range : +/- 2000dps
**
**
**  Return Value:
**      none
**
**  Errors:
**		none
**  Description:
**		The function sets the appropriate dps range bits in the CTRL_REG1_G register.
**
*/
void PModNav::setRangeG(unsigned char bRangeG)
{
    m_DPSRangeLSB = this->getGRangeLSB(bRangeG);
    this->setBitsInRegister(CS_AG, CTRL_REG1_G, MSK_RANGE_G, bRangeG, 3);
}

/* ------------------------------------------------------------ */
/*   PModNav::getRangeG()
**
**  Parameters:
**
**  Return Value:
**      float - returns the previously selected range from CTRL_REG1_G register
**
**  Errors:
**		none
**  Description:
**		The function reads the g range bits in the CTRL_REG1_G register and computes the range to be provided to the user.
**		The accepted argument values are between 0 and 3.If the value is not a valid one, the default value for range is set
*/
float PModNav::getRangeG()
{
    unsigned short readRange, gRange;
    readRange = this->getBitsInRegister(CS_AG, CTRL_REG1_G, 3, 2);
    switch(readRange)
    {
        case PAR_G_245DPS:
            gRange = 245;
            break;
        case PAR_G_500DPS:
            gRange = 500;
            break;
        case PAR_G_2kDPS:
            gRange = 2000;
            break;
        default:
            gRange = 245;
            break;
    }
    return gRange;
}

/* ------------------------------------------------------------ */
/*  PModNav::dataAvailableG()
**
**  Parameters:
**		none
**  Return Value:
**     unsigned char - returns the data available status in STATUS_REG register, for gyro
**
**  Errors:
**		none
**  Description:
**		The function reads the STATUS_REG register and returns the data available bit in it
**
*/
unsigned char PModNav::dataAvailableG()
{
    unsigned char status;
    this->readRegister(CS_AG, STATUS_REG, &status, 1);
    return ((status & (1<<1))>> 1);
}

/* ------------------------------------------------------------ */
/*   PModNav::ConfigIntG(unsigned char bIntGen, bool aoi, bool latch)
**
**   Parameters:
**		bIntGen	- The parameter indicating the interrupt generator. Can be one of the parameters from the following list
**			MSK_XLIE_G	 				1<<0
**			MSK_XHIE_G					1<<1
**			MSK_YLIE_G					1<<2
**			MSK_YHIE_G					1<<3
**			MSK_ZLIE_G					1<<4
**			MSK_ZHIE_G					1<<5
**		aoi - parameter indicating whether the interrupt generators are or-ed or and-ed together
**		latch - parameter that sets or not the latch interrupt request
**   Return Value:
**      none
**
**   Errors:
**		none
**
**   Description:
**		The function sets the interrupt threshold for each axis and also the duration of the enter/exit interrupt. Enables or dissables the
**		wait on duration before exiting interrupt.
**
*/
void PModNav::configIntG(unsigned char bIntGen, bool aoi, bool latch)
{
    unsigned char temp = bIntGen;
    if (aoi) temp |= 0x80;
    if (latch) temp |= 0x40;
    this->writeSPI(CS_AG, INT_GEN_CFG_G, temp);
}
/* ------------------------------------------------------------ */
/*  PModNav::setIntThresholdG(float thVal, unsigned short axis, bool drCntMode,unsigned char intDuration, bool wait)
**
**  Parameters:
**		thVal - Parameters containing the threshold value for one of the axes or for all
**		axis - parameter indicating the axis for which the threshold is set. It can be one of the following values:
**				X_AXIS			0
**				Y_AXIS			1
**				Z_AXIS			2
**		drCntMode - counter mode for interrupt
**		intDuration - parameter indicating the duration of the enter/exit interrupt
**		wait - parameter enabling or disabling the wait time before exiting the interrupt routine.
**  Return Value:
**      none
**
**  Errors:
**		none
**  Description:
**		The function sets the interrupt threshold for the selected or all axes, the counter mode for interrupt
**		and also the duration of the enter/exit interrupt. Enables or disables the wait on duration
**		before exiting interrupt.
**
*/
void PModNav::setIntThresholdG(float thVal, unsigned short axis, bool drCntMode,unsigned char intDuration, bool wait)
{
    unsigned char buffer[2];
    unsigned short bthVal;

    //convert the float value in g to raw accel data to be written in INT_GEN_THS_XH/XL/YH/YL/ZH/ZL_G registers
    bthVal = (unsigned short)(thVal/m_DPSRangeLSB);
    //split bytes
    buffer[0] = (bthVal & 0x7F00) >> 8;
    buffer[1] = (bthVal & 0x00FF);
    switch(axis)
    {
        case X_AXIS:
            //set the first bit, decrement or reset counter mode for interrupt
            buffer[0] |= drCntMode<<8;
            this->writeSPI(CS_AG, INT_GEN_THS_XH_G + (axis*2), buffer[0]);
            this->writeSPI(CS_AG, INT_GEN_THS_XH_G + 1 + (axis*2), buffer[1]);
            break;
        case Y_AXIS:
            this->writeSPI(CS_AG, INT_GEN_THS_XH_G + (axis*2), buffer[0]);
            this->writeSPI(CS_AG, INT_GEN_THS_XH_G + 1 + (axis*2), buffer[1]);
            break;
        case Z_AXIS:
            this->writeSPI(CS_AG, INT_GEN_THS_XH_G + (axis*2), buffer[0]);
            this->writeSPI(CS_AG, INT_GEN_THS_XH_G + 1 + (axis*2), buffer[1]);
            break;
        default:
            break;
    }
    // Write duration and wait to INT_GEN_DUR_G
    unsigned char temp;
    temp = (intDuration & 0x7F);
    if (wait) temp |= 0x80;
    this->writeSPI(CS_AG, INT_GEN_DUR_G, temp);

}

/*-------------------------------------------------------------*/
/*	XL+G Geographic Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*  PModNav::GetIntSrcXLG(unsigned char bInstMode)
**
**  Parameters:
**		bInstMode - parameter selecting between the two instruments, Accel and Gyro:
**			MODE_INST_A 	0 - accelerometer mode
**			MODE_INST_AG 	1 - Gyro
**  Return Value:
**      unsigned char - returns the content of the INT_GEN_SRC_XL or INT_GEN_SRC_G, depending of bInstMode parameter
**
**  Errors:
**		none
**
**  Description:
**		The function returns the source of interrupt for either accel or gyro instruments.
**
*/
unsigned char PModNav::getIntSrcXLG(unsigned char bInstMode)
{
    unsigned char intSrc;
    if (bInstMode==MODE_INST_A)
    {
        intSrc = this->readSPI(CS_AG, INT_GEN_SRC_XL);
        // Check if the IA_XL (interrupt active) bit is set
        if (intSrc & (1<<6))
        {
            return intSrc;
        }
        intSrc &=0x3F;
    }

    else if (bInstMode==MODE_INST_AG)
    {
        intSrc = this->readSPI(CS_AG, INT_GEN_SRC_G);
        // Check if the IA_G (interrupt active) bit is set
        if (intSrc & (1<<6))
        {
            return intSrc;
        }
        intSrc &=0x3F;
    }
}

/*-------------------------------------------------------------*/
/*	Magnetometer Specific Functions
/* ------------------------------------------------------------ */
/*   PModNav::ReadMag(short &MagX, short &MagY, short &MagZ)
**
**  Parameters:
**		&MagX	- the output parameter that will receive magnetometer value on X axis - 16 bits value
**		&MagY	- the output parameter that will receive magnetometer value on Y axis - 16 bits value
**		&MagZ	- the output parameter that will receive magnetometer value on Z axis - 16 bits value
**
**  Return Values:
**      none
**
**  Errors:
**		none
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the magnetometer.
**			-	It reads simultaneously the magnetic field value on three axes in a buffer of 6 bytes using the this->readRegister function
**			-	For each of the three axes, combines the two bytes in order to get a 16-bit value
**
*/
void PModNav::readMag(short &MagX, short &MagY, short &MagZ)
{
    unsigned char iMagX_L, iMagX_H, iMagY_L, iMagY_H, iMagZ_L, iMagZ_H;
    unsigned char status;
    unsigned char rgwRegVals[6];
    do
    {
        this->readRegister(CS_M, STATUS_REG_M, &status, 1);
    }
    while(status&0x08==0);
    //reads the bytes using the incremeting address functionality of the device.
    this->readRegister(CS_M, OUT_X_L_M, (unsigned char *)rgwRegVals, 6);
    iMagX_L = rgwRegVals[0];
    iMagX_H = rgwRegVals[1];
    iMagY_L = rgwRegVals[2];
    iMagY_H = rgwRegVals[3];
    iMagZ_L = rgwRegVals[4];
    iMagZ_H = rgwRegVals[5];
    //combines the read values for each axis to obtain the 16-bits values
    MagX = ((short)iMagX_H << 8) | iMagX_L;
    MagY = ((short)iMagY_H << 8) | iMagY_L;
    MagZ = ((short)iMagZ_H << 8) | iMagZ_L;
}
/* ------------------------------------------------------------ */
/*  PModNav::readMagGauss(float &MagXGauss, float &MagYGauss, float &MagZGauss)
**
**  Parameters:
**		&MagXGauss	- the output parameter that will receive magnetic value on X axis (in "Gauss")
**		&MagYGauss	- the output parameter that will receive magnetic value on Y axis (in "Gauss")
**		&MagZGauss	- the output parameter that will receive magnetic value on Z axis (in "Gauss")
**
**  Return Values:
**      none
**
**  Errors:
**		none
**  Description:
**		This function is the main function used for magnetic field values reading, providing the 3 current magnetometer values in �Gauss�.
**		It returns the gyro values measured on the three axes in "Gauss", using the raw values and the conversion function
**		For each of the three values, converts the 16-bit value to the value expressed in �Gauss�, considering the currently selected Gauss range
*/
void PModNav::readMagGauss(float &MagXGauss, float &MagYGauss, float &MagZGauss)
{
    short MagX, MagY, MagZ;

    this->readMag(MagX, MagY, MagZ);
    MagXGauss = this->convertReadingToValueGauss(MagX);
    MagYGauss = this->convertReadingToValueGauss(MagY);
    MagZGauss = this->convertReadingToValueGauss(MagZ);
}
/* ------------------------------------------------------------ */
/*  PModNav::convertReadingToValueGauss(short rawVal)
**
**  Parameters:
**		rawVal	- the 2 bytes containing the raw reading.
**
**  Return Values:
**      float - the value of the magnetic field in "gauss" corresponding to the 16 bits reading and the currently selected range
**
**  Errors:
**		none
**  Description:
**		Converts the value from the 16 bits reading to the float value (in gauss) corresponding to the magnetic field value, considering the current selected gauss range.
**
*/
float PModNav::convertReadingToValueGauss(short rawVal)
{
    float dResult = ((float)rawVal )* m_GaussRangeLSB;
    return dResult;
}

/* ------------------------------------------------------------ */
/*  PModNav::getMAGRangeLSB(unsigned char bRangeMAG)
**
**  Parameters:
**		unsigned char bRangeMAG	- the parameter specifying the gauss range. Can be one of the parameters from the following list:
**					0	PAR_MAG_4GAUSS	Parameter gauss range : +/- 4gauss
**					1	PAR_MAG_8GAUSS	Parameter gauss range : +/- 8gauss
**					2	PAR_MAG_12GAUSS	Parameter gauss range : +/- 12gauss
**					3	PAR_MAG_16GAUSS Parameter gauss range : +/- 16gauss
**
**  Return Value:
**      float  - returns the LSB unit value specific for each available range
**
**  Errors:
**		none
**  Description:
**		The function computes the range LSB based on the bRangeMAG parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "gauss" value.
**		If value is outside this range, the default value is set.
**
*/
float PModNav::getMAGRangeLSB(unsigned char bRangeMAG)
{
    float gRangeLSB;
    switch(bRangeMAG)
    {
        case PAR_MAG_4GAUSS:
            gRangeLSB = 0.00014;
            break;
        case PAR_MAG_8GAUSS:
            gRangeLSB = 0.00029;
            break;
        case PAR_MAG_12GAUSS:
            gRangeLSB = 0.00043;
            break;
        case PAR_MAG_16GAUSS:
            gRangeLSB = 0.00058;
            break;
        default:
            gRangeLSB = 0.00014;
            break;
    }
    return gRangeLSB;
}
/* ------------------------------------------------------------ */
/*   PModNav::setRangeMAG(unsigned char bRangeMAG)
**
**  Parameters:
**		bRangeMAG	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_MAG_4GAUSS	Parameter g range : +/- 4g
**					1	PAR_MAG_8GAUSS	Parameter g range : +/- 8g
**					2	PAR_MAG_12GAUSS	Parameter g range : +/- 12g
**					3	PAR_MAG_16GAUSS Parameter g range : +/- 16g
**
**  Return Value:
**		none
**
**  Errors:
**		none
**
**  Description:
**		The function sets the appropriate gauss range bits in the CTRL_REG2_M register.
**
*/
void PModNav::setRangeMAG(unsigned char bRangeMAG)
{
    m_GaussRangeLSB = this->getMAGRangeLSB(bRangeMAG);
    this->setBitsInRegister(CS_M, CTRL_REG2_M, MSK_RANGE_MAG, bRangeMAG, 5);
}
/* ------------------------------------------------------------ */
/*  PModNav::getRangeMAG()
**
**  Parameters:
**		none
**  Return Value:
**      unsigned char - returns the previously set range value
**
**  Errors:
**		none
**
**  Description:
**		The function reads the gauss range bits in the CTRL_REG2_M register and computes the range to be provided to the user.
**		The accepted argument values are between 0 and 3. If value is outside this range, the default value is set
**
*/
unsigned char PModNav::getRangeMAG()
{
    unsigned char readRange, gRange;
    readRange = this->getBitsInRegister(CS_M, CTRL_REG2_M, 5, 2);
    switch(readRange)
    {
        case PAR_MAG_4GAUSS:
            gRange = 4;
            break;
        case PAR_MAG_8GAUSS:
            gRange = 8;
            break;
        case PAR_MAG_12GAUSS:
            gRange = 12;
            break;
        case PAR_MAG_16GAUSS:
            gRange = 16;
            break;
        default:
            gRange = 4;
            break;
    }
    return gRange;
}
/* ------------------------------------------------------------ */
/*  PModNav::dataAvailableMAG(unsigned char axis)
**
**  Parameters:
**		axis - parameter indicating the axis for which the data availability is checked. It can be one of the values:
**			X_AXIS			0
**			Y_AXIS			1
**			Z_AXIS			2
**
**  Return Value:
**       unsigned char - returns the data available status in STATUS_REG_M register, for the selected axis
**
**  Errors:
**		none
**
**  Description:
**		The function reads the STATUS_REG_M register and returns the data available bit in it, for each of the axes
**
*/
unsigned char PModNav::dataAvailableMAG(unsigned char axis)
{
    unsigned char status;
    this->readRegister(CS_M, STATUS_REG_M, &status, 1);
    return ((status & (1<<axis)) >> axis);
}
/* ------------------------------------------------------------ */
/*  PModNav::configIntMAG(unsigned char bIntGen, unsigned char bActiveType, bool latch)
**
** 	Parameters:
**		bIntGen - interrupt generator sources. It can be one of the following:
**				MSK_ZIEN_MAG				1<<5
**				MSK_YIEN_MAG				1<<6
**				MSK_XIEN_MAG 				1<<7
**		bActiveType - interrupt active low or high parameter:
**				PAR_INT_ACTIVEHIGH		0
**				PAR_INT_ACTIVELOW		1
**		latch - parameter indicating the interrupt event is latched or not
**  Return Value:
**      none
**
**  Errors:
**		none
**	Description:
**		The function configures the interrupts for magnetometer instruments. It sets the interrupt generator for the three axes
**		It sets the active level, low or high for the interrupt event. It enables/disables the interrupt latching
**
*/
void PModNav::configIntMAG(unsigned char bIntGen, unsigned char bActiveType, bool latch)
{
    // Mask out non-generator bits (0-4)
    unsigned char config = (bIntGen & 0xE0);
    // IEA bit is 0 for active-low, 1 for active-high.
    if (bActiveType == PAR_INT_ACTIVEHIGH) config |= (1<<2);
    // IEL bit is 0 for latched, 1 for not-latched
    if (!latch) bIntGen |= (1<<1);
    // As long as we have at least 1 generator, enable the interrupt
    if (bIntGen != 0) config |= (1<<0);

    this->writeSPI(CS_M, INT_CFG_M, config);
}
/* ------------------------------------------------------------ */
/*  PModNav::setIntThresholdM(float thVal)
**
**  Parameters:
**		thVal - the threshold value set to all axes
**  Return Value:
**      none
**
**   Errors:
**		none
**
**   Description:
**		The function sets the interrupt threshold for the magnetometer instrument
*/
void PModNav::setIntThresholdM(float thVal)
{
    unsigned char buffer[2];
    unsigned short bthVal;
    //converts the float value in gauss to raw magnetic field data to be written in INT_THS_L_M/INT_THS_H_M registers
    bthVal = (unsigned short)(thVal/m_GaussRangeLSB);
    //split bytes
    //make sure the first bit of the High byte is 0, for correct functionality of the device
    buffer[0] = (bthVal & 0x7F00) >> 8;
    buffer[1] = (bthVal & 0x00FF);
    this->writeSPI(CS_M, INT_THS_H_M, buffer[0]);
    this->writeSPI(CS_M, INT_THS_L_M, buffer[1]);
}

/* ------------------------------------------------------------ */
/*   PModNav::getIntSrcMAG()
**
**  Parameters:
**		none
**  Return Value:
**      unsigned char - returns the interrupt sources for magnetometer instrument, after reading INT_SRC_M register
**
**  Errors:
**		none
**  Description:
**		The function returns the source of interrupt for magnetometer instrument.
**
*/
unsigned char PModNav::getIntSrcMAG()
{
    unsigned char intSrc;
    intSrc = this->readSPI(CS_M, INT_SRC_M);
    // Check if the INT (interrupt active) bit is set
    if (intSrc & (1<<0))
    {
        return (intSrc);// & 0xFE);
    }
    intSrc&=0xFE;
}
/* ------------------------------------------------------------ */
/*  PModNav::convMagToPolar(float mXGauss, float mYGauss, float mZGauss)
**
**  Parameters:
**		mXGauss, mYGauss, mZGauss - the magnetic field values for all the three axes
**  Return Value:
**      POLAR_T - returns the POLAR_T structure members values
**
**  Errors:
**		none
**  Description:
**		The function computes the R and D, polar coordinates of the magnetic field.
**		Updates the POLAR_T structure members D and R with the calculated values -  degrees of declination for D,
**		to further help indicate North, in compass functioning.
**
*/
PolarData PModNav::convMagToPolar(float mXGauss, float mYGauss, float mZGauss)
{
    //update the POLAR_T structure member R with the field resultant
    coord.R = sqrt(pow(mXGauss,2)+ pow(mYGauss,2)+pow(mZGauss,2));
    //calculate the declination using two of the axes values, X and Y and reduce to first quadrant the values
    if (mXGauss == 0)
        coord.D = (mYGauss < 0) ? 90 : 0;
    else
        coord.D = atan2(mYGauss,mXGauss)*180/PI;
    if (coord.D > 360)
    {
        coord.D -= 360;
    }

    else if (coord.D<0)
    {
        coord.D+=360;
    }
}

/*-------------------------------------------------------------*/
/*	Altimeter Specific Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*  PModNav::ComputePref(float altitudeMeters)
**
**  Parameters:
**		altitudeMeters	- the parameter used to calibrate the altitude computing, is considered known for
**		the wanted location
**
**  Return Values:
**      void
**
**  Errors:
**
**  Description:
**		This function provides the reference pressure computed with a known altitude for the given location
**			-	it performs a pressure reading, then computes the Reference pressure using the altitude parameter.
**		It needs to be called once for the correct operation of the altitude function, all the following pressure readings
**		being affected by it.
**		This is needed because the current altitude is also affected by the current sea level air pressure, while the barometric
**		pressure formula used to compute altitude is considering the sea level pressure constant at all times.
**
*/
void PModNav::computePref(float altitudeMeters)
{
    float Pinit, altCorrected;
    Pinit = this->readPressurehPa();  /* Measured Pressure  */

    float temp = 1 - (altitudeMeters*3.2808/145366.45);
    Pref= Pinit/(pow(temp,1/0.190284));
}

/*  PModNav::readPressure()
**
**  Parameters:
**		none
**
**  Return Values:
**      int - returns the raw measured pressure value later used for converting it in hPa
**
**  Errors:
**		none
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the barometer instrument.
**			-	It reads the pressure value from the low, middle and high registers using the this->readRegister function
**			-	combines the three registers to obtain a 24-bit value for the pressure
*/
int PModNav::readPressure()
{
    unsigned char iPress_XL, iPress_L, iPress_H;
    unsigned char rgwRegVals[3];
    int32_t dataPress;
    this->readRegister(CS_ALT, PRESS_OUT_XL, (unsigned char *)rgwRegVals, 3);
    iPress_XL = rgwRegVals[0];
    iPress_L = rgwRegVals[1];
    iPress_H = rgwRegVals[2];
    dataPress = (iPress_H << 16)|iPress_L<<8|iPress_XL;
    return dataPress;
}
/* ------------------------------------------------------------ */
/*		PModNav::readPressurehPa()
**
**   Parameters:
**		none
**
**   Return Values:
**      float - returns the value of measured pressure in hPa
**
**   Errors:
**		none
**
**   Description:
**		This function provides the pressure in hPa
**
*/
float PModNav::readPressurehPa()
{
    unsigned int dataRawFull;
    dataRawFull = this->readPressure();
    //check if there is a negative value
    if (dataRawFull & 0x00800000){
        dataRawFull |= 0xFF000000;
    }
    hPa = dataRawFull/4096;
    return hPa;
}
/* ------------------------------------------------------------ */
/*  PModNav::convPresToAltM(float hPa)
**
**   Parameters:
**		float hPa	- parameter representing the value of pressure in hPa
**
**   Return Values:
**      float - it returns the current altitude based on the measured pressure and the previously computed reference pressure
**
**   Errors:
**		none
**
**   Description:
**		This function converts the current pressure to altitude using the previously computed Pref as reference pressure.
**		The Pref is computed once and used for further calculations of the altitude. The value returned is in meters
**
*/
float PModNav::convPresToAltM(float hPa)
{
    float altMeters;
    float altCorrected = ((1-pow(hPa/Pref,0.190284))*145366.45);
    altMeters = altCorrected*0.3048;
    return altMeters;
}
/* ------------------------------------------------------------ */
/*  PModNav::convPresToAltF(float hPa)
**
**   Parameters:
**		float hPa	- parameter representing the value of pressure in hPa
**
**   Return Values:
**      float - returns the value of the altitude in feet
**
**   Errors:
**		none
**   Description:
**		This function performs the conversion from meters to feet and returns the value of altitude in feet
**
**
*/
float PModNav::convPresToAltF(float hPa)
{
    float altfeet;
    altfeet = ((1-pow(hPa/Pref,0.190284))*145366.45);
    return altfeet;
}
/* ------------------------------------------------------------ */
/*    PModNav::readTempC()
**
**   Parameters:
**		none
**
**   Return Values:
**      float - the function returns the value of the read temperature in degrees Celsius
**
**   Errors:
**		none
**   Description:
**		Reads and computes the temperature in degrees Celsius
**
*/
float PModNav::readTempC()
{
    unsigned char rgwRegVals[2], tempL, tempH;
    this->readRegister(CS_ALT, TEMP_OUT_L,  (unsigned char *)rgwRegVals,2);
    tempL = rgwRegVals[0];
    tempH = rgwRegVals[1];
    short temp = (short)tempH <<8 | tempL;
    //datasheet formula used for converting to temperature in degrees Celsius from raw values
    tempC = 42.5 +(temp * 0.002083);
    return tempC;
}
/* ------------------------------------------------------------ */
/*  PModNav::convTempCToTempF(float tempC)
**
**  Parameters:
**		tempC	- parameter representing the value of temperature expressed in degrees Celsius
**
**  Return Values:
**      float - returns the value of the temperature in degrees Fahrenheit
**
**   Errors:
**
**   Description:
**		This function performs the conversion from Celsius to Fahrenheit degrees and returns the value of temperature in F
**
*/
float PModNav::convTempCToTempF(float tempC)
{
    float tempF = 32 +(tempC * 1.8);
    return tempF;
}
/* ------------------------------------------------------------ */
/*    PModNav::dataAvailableALT()
**
**   Parameters:
**		none
**
**   Return Value:
**      unsigned char - returns the data available status in STATUS_REG register, for altimeter
**
**   Errors:
**
**   Description:
**		The function reads the STATUS_REG register and returns the data available bit in it
**
*/
unsigned char PModNav::dataAvailableALT()
{
    unsigned char status;
    this->readRegister(CS_ALT, STATUS_REG, &status, 1);
    return ((status & (1<<1)) >> 1);
}
/* ------------------------------------------------------------ */
/*    PModNav::tempAvailableALT()
**
**   Parameters:
**		none
**
**   Return Value:
**      unsigned char - returns the temperature available status in STATUS_REG register, for altimeter
**
**   Errors:
**		none
**   Description:
**		The function reads the STATUS_REG register and returns the temperature available bit in it
**
*/
unsigned char PModNav::tempAvailableALT()
{
    unsigned char status;
    this->readRegister(CS_ALT, STATUS_REG, &status, 1);
    return (status & (1<<0));
}
/* ------------------------------------------------------------ */
/*  PModNav::configIntALT(unsigned char bIntGen, unsigned char bActiveType, unsigned char bOutputType, unsigned char dataSignalVal,
**                        bool intEnable, bool latch, unsigned char intLevel)
**
**   Parameters:
**		bIntGen - interrupt generator sources. It can be one of the following:
**				 MSK_INT_F_EMPTY			1<<3
**				 MSK_INT_F_FTH				1<<2
**				 MSK_INT_F_OVR				1<<1
**				 MSK_INT_DRDY				1<<0
**		bActiveType - interrupt active low or high parameter:
**				PAR_INT_ACTIVEHIGH		0
**				PAR_INT_ACTIVELOW		1
**		dataSignalVal - INT_S bits value, representing the interrupt configurations, data signal, pressure high, low, or high and low
**			it can be one of the following:
**				MSK_INT_P_HIGH				0x01
**				MSK_INT_P_LOW				0x02
**				MSK_INT_P_LOW_HIGH			0x03
**		intEnable - enable interrupts parameter
**		bOutputType - output type parameter, one of the following:
**				PAR_INT_OPENDRAIN			1
**				PAR_INT_PUSHPULL			0
**		latch - parameter indicating the interrupt event is latched or not
**		intLevel - set the level active interrupt for differential pressure value, on high or low
**				MSK_INT_LEVEL_HIGH				1<<0
**				MSK_INT_LEVEL_LOW				1<<1
**  Return Value:
**      none
**
**  Errors:
**		none
**	Description:
**		The function configures the interrupts for altimeter instruments. It sets the interrupt generator for the three axes
**		It sets the active level, low or high for the interrupt event, interrupt configurations, output type.
**		It enables/disables the interrupt latching
**
*/
void PModNav::configIntALT(unsigned char bIntGen, unsigned char bActiveType, unsigned char bOutputType, unsigned char dataSignalVal,
                           bool intEnable, bool latch, unsigned char intLevel)
{
    //enable interrupts in CTRL_REG1 register
    this->setRegisterBits(CS_ALT, CTRL_REG1, MSK_DIFF_EN_ALT, intEnable);
    //mask out the reserved bits in CTRL_REG3 register
    unsigned char config = (bIntGen & 0xC3);
    // INT_H_L bit is 1 for active-low, 0 for active-high.
    if (bActiveType == PAR_INT_ACTIVELOW)
    {
        config |= (1<<7);
    }
    // PP_OD bit is 1 for open drain, 0 for push pull.
    if (bOutputType == PAR_INT_OPENDRAIN)
    {
        config |= (1<<6);
    }
    config|=dataSignalVal;
    this->writeSPI(CS_ALT, CTRL_REG3, config);
    config = bIntGen;
    this->writeSPI(CS_ALT, CTRL_REG4, config);
    if (!latch) config |= (1<<2);
    config|= intLevel;
    this->writeSPI(CS_ALT, INTERRUPT_CFG, config);

}
/* ------------------------------------------------------------ */
/*  PModNav::setIntThresholdALT(float thVal)
**
**   Parameters:
**		thVal - the interrupt threshold parameter for alt instrument
**   Return Value:
**      none
**
**   Errors:
**
**   Description:
**		The function sets the interrupt threshold for the altimeter instrument
**
*/
void PModNav::setIntThresholdALT(float thVal)
{
    unsigned char buffer[2];
    unsigned short bthVal;
    //converts the float value in gauss to raw magnetic field data to be written in INT_THS_L_M/INT_THS_H_M registers
    bthVal = (unsigned short)(thVal*4096);
    //split bytes
    //make sure the first bit of the High byte is 0, for correct functionality of the device
    buffer[0] = (bthVal & 0xFF00) >> 8;
    buffer[1] = (bthVal & 0x00FF);
    this->writeSPI(CS_ALT, THS_P_H, buffer[0]);
    this->writeSPI(CS_ALT, THS_P_L, buffer[1]);
}

/* ------------------------------------------------------------ */
/*    PModNav::getIntSrcALT()
**
**  Parameters:
**
**  Return Value:
**     unsigned char - parameter storing the interrupt source register content
**
**  Errors:
**		none
**  Description:
**		The function gets the interrupt sources by reading the INT_SOURCE register
**
*/
unsigned char PModNav::getIntSrcALT()
{
    unsigned char intSrc;
    intSrc = this->readSPI(CS_ALT, INT_SOURCE);

    // Check if the INT (interrupt active) bit is set
    if (intSrc & (1<<2))
    {
        return intSrc;
    }
    intSrc &= 0x03;
}


/*-------------------------------------------------------------*/
/*	Geographic Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*  PModNav::setODR(unsigned char bInstMode, unsigned char odrVal)
**
**    Parameters:
**			bInstMode - parameter representing the instrument to be affected
**			odrVal - the parameter specifying the ODR value for each instrument. Can be one of the parameters from the following list:
**					----for Accelerometer
**					0	ODR_XL_PWR_DWN	Parameter ODR value: power down the device
**					1	ODR_XL_10_HZ	Parameter ODR value: 10 Hz
**					2	ODR_XL_50_HZ	Parameter ODR value: 50 Hz
**					3	ODR_XL_119_HZ	Parameter ODR value: 119 Hz
**					4	ODR_XL_238_HZ	Parameter ODR value: 238 Hz
**					5	ODR_XL_476_HZ	Parameter ODR value: 476 Hz
**					6	ODR_XL_952_HZ	Parameter ODR value: 952 Hz
**					7	ODR_XL_NA		Parameter ODR value: not defined
**					----for Gyro
**					0	ODR_G_PWR_DWN	Parameter ODR value: power down the device
**					1	ODR_G_14_9HZ	Parameter ODR value: 14.9 Hz
**					2	ODR_G_59_5HZ	Parameter ODR value: 59.5 Hz
**					3	ODR_G_119HZ		Parameter ODR value: 119 Hz
**					4	ODR_G_238HZ		Parameter ODR value: 238 Hz
**					5	ODR_G_476HZ		Parameter ODR value: 476 Hz
**					6	ODR_G_952HZ		Parameter ODR value: 952 Hz
**					7	ODR_G_NA		Parameter ODR value: not defined
**					----for Magnetometer
**					0	ODR_M_0_625HZ	Parameter ODR value: 0.625 HZ
**					1	ODR_M_1_25HZ	Parameter ODR value: 1.25 Hz
**					2	ODR_M_2_5HZ		Parameter ODR value: 2.5 Hz
**					3	ODR_M_5HZ		Parameter ODR value: 5 Hz
**					4	ODR_M_10HZ		Parameter ODR value: 10 Hz
**					5	ODR_M_20HZ		Parameter ODR value: 20 Hz
**					6	ODR_M_40HZ		Parameter ODR value: 40 Hz
**					7	ODR_M_80HZ		Parameter ODR value: 80 HZ
**   Return Value:
**      none
**
**   Errors:
**		none
**
**   Description:
**		The function sets the ODR value for the selected instrument, in the corresponding register.
**		The accepted argument values are between 0 and 7.
**
*/
void PModNav::setODR(unsigned char bInstMode, unsigned char odrVal)
{
    unsigned char setODR;
    switch(bInstMode)
    {
        case MODE_INST_A:
            //set ODR for accelerometer instrument when used in single mode
            this->setBitsInRegister(CS_AG, CTRL_REG6_XL, MSK_ODR_XL, odrVal, 5);
            break;
        case MODE_INST_AG:
            //set ODR for gyro and accel instruments when used together
            this->setBitsInRegister(CS_AG, CTRL_REG1_G, MSK_ODR_G, odrVal, 5);
            break;
        case MODE_INST_MAG:
            //set ODR for magnetometer instrument
            this->setBitsInRegister(CS_M, CTRL_REG6_XL, MSK_ODR_MAG, odrVal, 3);
            break;
        case MODE_INST_ALT:
            //set ODR for altimeter instrument
            this->setBitsInRegister(CS_ALT, CTRL_REG1, MSK_ODR_ALT, odrVal, 4);
            break;
        default:
            break;
    }
}
/* ------------------------------------------------------------ */
/*   PModNav::getODRRaw(unsigned char bInstMode)
**
**  Parameters:
**		bInstMode - parameter representing the instrument to be affected
**  Return Value:
**      unsigned char - returns the ODR raw value for the selected instrument, expressed in hexa
**
**  Errors:
**		none
**
**  Description:
**		The function sets the ODR value for the selected instrument, in the corresponding register.
**		The accepted argument values are between 0 and 3.
**
*/
unsigned char PModNav::getODRRaw(unsigned char bInstMode)
{
    unsigned char getODR;
    switch(bInstMode)
    {
        case MODE_INST_A:
            getODR = this->getBitsInRegister(CS_AG, CTRL_REG6_XL, 5, 3);
            break;
        case MODE_INST_AG:
            getODR = this->getBitsInRegister(CS_AG, CTRL_REG1_G, 5, 3);
            break;
        case MODE_INST_MAG:
            getODR = this->getBitsInRegister(CS_M, CTRL_REG6_XL, 3, 3);
            break;
        case MODE_INST_ALT:
            getODR = this->getBitsInRegister(CS_ALT, CTRL_REG1, 4, 3);
            break;
        default:
            break;
    }
    return getODR;
}

/* ------------------------------------------------------------ */
/*   PModNav::getODR(unsigned char bInstMode)
**
**   Parameters:
**		bInstMode - parameter representing the instrument to be affected
**   Return Value:
**      float - returns the ODR value of the selected instrument, based on the ODR bits reading from the corresponding register
**
**   Errors:
**		none
**
**   Description:
**		The function reads the ODR bits from each instrument register and computes the real ODR to be provided to the user.
**
**
*/
float PModNav::getODR(unsigned char bInstMode)
{
    unsigned char odrRead;
    float odrFinal;
    if (bInstMode ==MODE_INST_A)
    {
        odrRead = this->getODRRaw(MODE_INST_A);
        switch (odrRead)
        {
            case ODR_XL_PWR_DWN:
                odrFinal = 0;
                break;
            case ODR_XL_10HZ:
                odrFinal = 10;
                break;
            case ODR_XL_50HZ:
                odrFinal = 50;
                break;
            case ODR_XL_119HZ:
                odrFinal = 119;
                break;
            case ODR_XL_238HZ:
                odrFinal = 238;
                break;
            case ODR_XL_476HZ:
                odrFinal = 476;
                break;
            case ODR_XL_952HZ:
                odrFinal = 952;
                break;
            case ODR_XL_NA:
                odrFinal = -1;
                break;
            default:
                odrFinal = 0;
                break;
        }
    }
        //get odr for accel+gyro
    else if (bInstMode ==MODE_INST_AG)
    {
        odrRead = this->getODRRaw(MODE_INST_AG);
        switch (odrRead)
        {
            case ODR_G_PWR_DWN:
                odrFinal = 0;
                break;
            case ODR_G_14_9HZ:
                odrFinal = 14.9;
                break;
            case ODR_G_59_5HZ:
                odrFinal = 59.5;
                break;
            case ODR_G_119HZ:
                odrFinal = 119;
                break;
            case ODR_G_238HZ:
                odrFinal = 238;
                break;
            case ODR_G_476HZ:
                odrFinal = 476;
                break;
            case ODR_G_952HZ:
                odrFinal = 952;
                break;
            case ODR_G_NA:
                odrFinal = -1;
                break;
            default:
                odrFinal = 0;
                break;
        }
    }
        //get odr for magnetometer
    else if (bInstMode ==MODE_INST_MAG)
    {
        odrRead = this->getODRRaw(MODE_INST_MAG);
        switch (odrRead)
        {
            case ODR_M_0_625HZ:
                odrFinal = 0.625;
                break;
            case ODR_M_1_25HZ:
                odrFinal = 1.25;
                break;
            case ODR_M_2_5HZ:
                odrFinal = 2.5;
                break;
            case ODR_M_5HZ:
                odrFinal = 5;
                break;
            case ODR_M_10HZ:
                odrFinal = 10;
                break;
            case ODR_M_20HZ:
                odrFinal = 20;
                break;
            case ODR_M_40HZ:
                odrFinal = 40;
                break;
            case ODR_M_80HZ:
                odrFinal = 80;
                break;
            default:
                odrFinal = 10;
                break;
        }
    }
        //get odr for altimeter
    else if (bInstMode ==MODE_INST_ALT)
    {
        odrRead = this->getODRRaw(MODE_INST_ALT);
        switch (odrRead)
        {
            case ODR_ALT_ONE_SHOT:
                odrFinal = 0;
                break;
            case ODR_ALT_1HZ:
                odrFinal = 1;
                break;
            case ODR_ALT_7HZ:
                odrFinal = 7;
                break;
            case ODR_ALT_12_5HZ:
                odrFinal = 12.5;
                break;
            case ODR_ALT_25HZ:
                odrFinal = 25;
                break;
            default:
                odrFinal = 0;
                break;
        }
    }
    else odrFinal = -1;
    return odrFinal;
}
/*-------------------------------------------------------------*/
/*	FIFO Functions
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*   PModNav::fifoEnable(char ssPin, bool fEnable)
**
**  Parameters:
**		csPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**		fEnable - the parameter used to enable or disable the FIFO
**
**   Return Value:
**      none
**
**   Errors:
**		none
**   Description:
**		The function enables or disables FIFO by writing FIFO_EN bit in CTRL_REG9 register for Accel/Gyro, or CTRL_REG2 for Altimeter instrument
**
*/
void PModNav::fifoEnable(GPIO_PIN csPin, bool fEnable)
{
    unsigned char temp;
    if(csPin == CS_AG)
    {
        this->readRegister(CS_AG, CTRL_REG9, &temp, 1);
        if (fEnable) temp |= (1<<1);
        else temp &= ~(1<<1);
        this->writeSPI(CS_AG, CTRL_REG9, temp);
    }
    else
    {
        if(csPin == CS_ALT)
        {
            this->readRegister(CS_ALT, CTRL_REG2, &temp, 1);
            if (fEnable) temp |= (1<<6);
            else temp &= ~(1<<6);
            this->writeSPI(CS_ALT, CTRL_REG2, temp);
        }
    }
}
/* ------------------------------------------------------------ */
/*   PModNav::SetFIFO(int8_t ssPin, unsigned char parFIFOMode, unsigned char FIFOThreshold);
**
**  Parameters:
**		ssPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**		parFIFOMode - the parameter specifying the FIFO mode for each instrument. Can be one of the parameters from the following list:
**					----for Accelerometer and Gyro instruments, when working together
**					0	FIFO_MODE_XL_G_BYPASS			Parameter FIFO mode value: bypass
**					1	FIFO_MODE_XL_G_FIFO				Parameter FIFO mode value: FIFO normal
**					3	FIFO_MODE_XL_G_CONTINUOUS_FIFO	Parameter FIFO mode value: continuous to FIFO
**					4	FIFO_MODE_XL_G_BYPASS_CONTINUOUS		Parameter FIFO mode value: bypass to continuous
**					6	FIFO_MODE_XL_G_CONTINUOUS		Parameter FIFO mode value: continuous
**					----for Altimeter instrument
**					0	FIFO_MODE_ALT_BYPASS			Parameter FIFO mode value: bypass
**					1	FIFO_MODE_ALT_FIFO				Parameter FIFO mode value: FIFO normal
**					2	FIFO_MODE_ALT_STREAM			Parameter FIFO mode value: stream mode
**					3	FIFO_MODE_ALT_STREAM_TO_FIFO	Parameter FIFO mode value: stream to fifo
**					4	FIFO_MODE_ALT_BYPASS_TO_STREAM	Parameter FIFO mode value: bypass to stream
**					6	FIFO_MODE_ALT_MEAN				Parameter FIFO mode value: mean mode
**					7	FIFO_MODE_ALT_BYPASS_TO_FIFO	Parameter FIFO mode value: bypass to fifo mode
**		FIFOThreshold - FIFO threshold level setting. Can be one of the parameters from the following list:
**					----for Accelerometer and Gyro instruments, when working together. Any value from 0-0x1F is acceptable
**					0			Parameter FIFO mode mean value: 0 samples
**					.
**					.
**					.
**					32			Parameter FIFO mode mean value: 32 samples
**					----for Altimeter instrument
**					0	FIFO_MODE_MEAN_ALT_2SAMPLES		Parameter FIFO mean mode value: 2 samples
**					1	FIFO_MODE_MEAN_ALT_4SAMPLES		Parameter FIFO mean mode value: 4 samples
**					2	FIFO_MODE_MEAN_ALT_8SAMPLES		Parameter FIFO mean mode value: 8 samples
**					3	FIFO_MODE_MEAN_ALT_16SAMPLES	Parameter FIFO mean mode value: 16 samples
**					4	FIFO_MODE_MEAN_ALT_32SAMPLES	Parameter FIFO mean mode value: 32 samples
**   Return Value:
**      none
**
**   Errors:
**		none
**   Description:
**		The function sets the FIFO control mode and threshold in FIFO_CTRL register for the accel+gyro instruments and for altimeter instrument
**		The magnetometer instrument does not contain FIFO
**
*/
void PModNav::setFIFO(GPIO_PIN csPin, unsigned char parFIFOMode, unsigned char FIFOThreshold)
{
    if(csPin == CS_AG)
    {

        this->setBitsInRegister(CS_AG, FIFO_CTRL, MSK_FIFO_CTL_MODE, parFIFOMode, 5);
        this->setBitsInRegister(CS_AG, FIFO_CTRL, MSK_FIFO_THS, FIFOThreshold, 0);
    }
    else
    {
        if(csPin == CS_ALT)
        {
            this->setBitsInRegister(CS_ALT, FIFO_CTRL, MSK_FIFO_CTL_MODE, parFIFOMode, 5);
            this->setBitsInRegister(CS_ALT, FIFO_CTRL, MSK_FIFO_THS, FIFOThreshold, 0);
        }
    }
}
/* ------------------------------------------------------------ */
/*   PModNav::GetFIFOMode(int8_t ssPin)
**
**  Parameters:
**		ssPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**  Return Value:
**     unsigned char - returns the FIFO mode bits from FIFO_CTRL register for either Accel/Gyro or Altimeter instrument
**
**  Errors:
**		none
**  Description:
**		The function reads the FIFO mode bits in the FIFO_CTRL register for accel and gyro and Altimeter instruments
**		and returns their value.
**
*/
unsigned char PModNav::getFIFOMode(GPIO_PIN csPin)
{
    unsigned char getFIFO;
    if(csPin == CS_AG)
    {
        getFIFO = this->getBitsInRegister(CS_AG, FIFO_CTRL, 5, 3);
    }
    else
    {
        if(csPin == CS_ALT)
        {
            getFIFO = this->getBitsInRegister(CS_ALT, FIFO_CTRL, 5, 3);
        }
    }
    return getFIFO;
}

/* ------------------------------------------------------------ */
/*  PModNav::getFIFOThs(char ssPin)
**
**  Parameters:
**		csPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**	Return Value
**		unsigned char - specifies the FIFO mean number of samples for each instrument. Can be one of the parameters from the following list:
**					----for Accelerometer and Gyro instruments, when working together
**					0			Parameter FIFO mode mean value: 0 samples
**					.
**					.
**					.
**					32			Parameter FIFO mode mean value: 32 samples
**					----for Altimeter instrument
**					0	FIFO_MODE_MEAN_ALT_2SAMPLES		Parameter FIFO mean mode value: 2 samples
**					1	FIFO_MODE_MEAN_ALT_4SAMPLES		Parameter FIFO mean mode value: 4 samples
**					2	FIFO_MODE_MEAN_ALT_8SAMPLES		Parameter FIFO mean mode value: 8 samples
**					3	FIFO_MODE_MEAN_ALT_16SAMPLES	Parameter FIFO mean mode value: 16 samples
**					4	FIFO_MODE_MEAN_ALT_32SAMPLES	Parameter FIFO mean mode value: 32 samples
**
**  Errors:
**		none
**
**  Description:
**		The function returns the FIFO mean/threshold mode bits in FIFO_CTRL register for the accel+gyro instruments or for altimeter instrument
**		The magnetometer instrument does not contain FIFO
**
*/
unsigned char PModNav::getFIFOThs(GPIO_PIN csPin)
{
    unsigned char bFifoThs;
    if(csPin == CS_AG)
    {
        bFifoThs = this->getBitsInRegister(CS_AG, FIFO_CTRL, 0, 5);
    }
    else
    {
        if(csPin == CS_ALT)
        {
            bFifoThs = this->getBitsInRegister(CS_ALT, FIFO_CTRL, 0, 5);
        }
    }
    return bFifoThs;
}
/* ------------------------------------------------------------ */
/*   PModNav::getFIFOStatus(char csPin)
**
**  Parameters:
**		csPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**  Return Value:
**     unsigned char - returns the FIFO status FIFO_STATUS or FIFO_SRC register, depending on the selected instrument
**
**  Errors:
**		none
**  Description:
**		The function reads the FIFO_SRC register for accel and gyro and FIFO_STATUS register for Altimeter instrument
**		and returns their value.
**
*/
unsigned char PModNav::getFIFOStatus(GPIO_PIN csPin)
{
    unsigned char getFIFOSts;
    if(csPin == CS_AG)
    {
        getFIFOSts = this->readSPI(CS_AG, FIFO_SRC);
    }
    else
    {
        if(csPin == CS_ALT)
        {
            getFIFOSts = this->readSPI(CS_ALT, FIFO_STATUS);
        }
    }
    return getFIFOSts;
}


void PModNav::writeRegister(GPIO_PIN csPIN, unsigned char bAddr, unsigned char *buf, unsigned char size) {
    int ib;

    if(csPIN == CS_AG)
    {
        // make SS active.
        GPIO::writeGPIOValue(csPIN, LOW);
        unsigned char addr = (bAddr) | 0x80;
        // send first byte indicating the operation will be reading from SPI
        spi.writeSPI(&addr, 1);
        // write buf
        spi.writeSPI(buf, size);
        // make SS inactive
        GPIO::writeGPIOValue(csPIN, HIGH);
    }
    else if((csPIN == CS_M) || (csPIN == CS_ALT))
    {
        // make SS active
        GPIO::writeGPIOValue(csPIN, LOW);
        // send first byte indicating the operation will be reading from SPI
        unsigned char addr = (bAddr) | 0xC0;
        spi.writeSPI(&addr, 1);
        // write buf
        spi.writeSPI(buf, size);
        // make SS inactive
        GPIO::writeGPIOValue(csPIN, HIGH);
    }
}

void PModNav::readRegister(GPIO_PIN csPIN, unsigned char bAddr, unsigned char *buf, unsigned char size) {
    int ib;

    if(csPIN == CS_AG)
    {
        // make SS active
        GPIO::writeGPIOValue(csPIN, LOW);
        //write first byte indicating the operation will be writing
        unsigned char addr = (bAddr) | 0x00;
        spi.readSPI(&addr, 1, buf, size);
        GPIO::writeGPIOValue(csPIN, HIGH);
    }
    else if((csPIN == CS_M) || (csPIN == CS_ALT))
    {
        // make SS active
        GPIO::writeGPIOValue(csPIN, LOW);
        //write first byte indicating the operation will be writing
        unsigned char addr = (bAddr) | 0x40;
        spi.readSPI(&addr, 1, buf, size);
        GPIO::writeGPIOValue(csPIN, HIGH);
    }

}

void PModNav::writeSPI(GPIO_PIN csPIN, unsigned char bAddr, unsigned char bVal) {
    // make SS active
    GPIO::writeGPIOValue(csPIN, LOW);
    //write first byte indicating the operation will be writing
    unsigned char buf[2] = {(unsigned char)(bAddr | 0x00), bVal};
    spi.writeSPI(buf, bVal);
    GPIO::writeGPIOValue(csPIN, HIGH);
}

unsigned char PModNav::readSPI(GPIO_PIN csPIN, unsigned char bAddr) {
    unsigned char value;
    // make SS active
    GPIO::writeGPIOValue(csPIN, LOW);
    bAddr = bAddr | 0x80;
    //send first byte indicating the operation will be reading
    spi.readSPI(&bAddr, 1, &value, 1);
    // make SS inactive
    GPIO::writeGPIOValue(csPIN, LOW);
    return value;
}

/*-------------------------------------------------------------*/
/*	Bits Specific Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*   PModNav::PModNav::setBitsInRegister(char csPin, unsigned char bRegAddr, unsigned char bMask, unsigned char bValue, unsigned char startBit)
**
**   Parameters:
**		ssPin				- instrument selection for chip select: AG/MAG/ALT
**		bRegAddr		 	- the address of the register whose bits are set
**		bMask				- the mask indicating which bits are affected
**		bValue				- the byte containing bits values
**		startBit			- start bit of the bits group to be set in register
**
**   Return Values:
**       none
**   Errors:
**		none
**   Description:
**		This function sets the value of some bits (corresponding to the bMask) of a register (indicated by bRegAddr) to the value of the corresponding bits from another byte (indicated by bValue)
**		starting from the position indicated by startBit.
**
*/
void PModNav::setBitsInRegister(GPIO_PIN csPin, unsigned char bRegAddr, unsigned char bMask, unsigned char bValue, unsigned char startBit)
{
    unsigned char bRegValue, shiftedValue;
    shiftedValue = (bValue << startBit);
    readRegister(csPin, bRegAddr, &bRegValue, 1);
    // register value: mask out the bits from the mask
    bRegValue &= ~bMask;
    // value: mask out the values outside the mask
    shiftedValue &= bMask;
    // combine the value with the masked register value
    bRegValue |= (shiftedValue & bMask);
    writeRegister(csPin, bRegAddr, &bRegValue, 1);
}

/* ------------------------------------------------------------ */
/*  PModNav::getBitsInRegister(char csPin, unsigned char bRegAddr, unsigned char startBit, unsigned char noBits)
**
**  Parameters:
**		ssPin				- instrument selection for chip select: AG/MAG/ALT
**		bRegAddr		 	- the address of the register whose bits are set
**		startBit			- start bit of the bits group to be set in register
**		noBits				- number of bits starting from start bit, to be read
**
**   Return Values:
**      none
**   Errors:
**		none
**   Description:
**		This function gets the value of some bits (given by the startBts and noBits parameters) of a register (indicated by bRegisterAddress).
**
*/
unsigned char PModNav::getBitsInRegister(GPIO_PIN csPin, unsigned char bRegAddr, unsigned char startBit, unsigned char noBits)
{
    unsigned char bRegValue, bResult, bMask;
    this->readRegister(csPin, bRegAddr, &bRegValue, 1);
    bMask = ((1<<noBits)-1)<< startBit;
    bResult = (bRegValue & bMask) >> startBit;

    return bResult;
}
/* ------------------------------------------------------------ */
/*   PModNav::setRegisterBits(char csPin, unsigned char bRegAddr, unsigned char bMask, bool fValue)
**
**   Parameters:
**		csPin				- instrument selection for chip select: AG/MAG/ALT
**		bRegAddr		 	- the address of the register whose bits are set
**		bMask				- the mask indicating which bits are affected
**		fValue				- 1 if the bits are set or 0 if their bits are reset
**
**   Return Values:
**      none
**   Errors:
**		none
**   Description:
**		This function sets the value of some bits (corresponding to the bMask) of a register (indicated by bRegAddr) to 1 or 0 (indicated by fValue).
**
*/
void PModNav::setRegisterBits(GPIO_PIN csPin, unsigned char bRegAddr, unsigned char bMask, bool fValue)
{
    unsigned char bRegValue;
    this->readRegister(csPin, bRegAddr, &bRegValue, 1);
    if(fValue)
    {
        // set 1 value to the values that are 1 in the mask
        bRegValue |= bMask;
    }
    else
    {
        // set 0 value to the values that are 1 in the mask
        bRegValue &= ~bMask;
    }
    this->writeRegister(csPin, bRegAddr, &bRegValue,1);
}


