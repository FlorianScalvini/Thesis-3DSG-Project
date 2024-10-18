//
// Created by ubuntu on 25/11/22.
//

#ifndef OUTDOORNAV_ADAFRUIT_BNO055_CONSTANT_H
#define OUTDOORNAV_ADAFRUIT_BNO055_CONSTANT_H



/** BNO055 Address A **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/** Offsets registers **/
#define NUM_BNO055_OFFSET_REGISTERS (22)

/** A structure to represent offsets **/
typedef struct {
    unsigned short accel_offset_x; /**< x acceleration offset */
    unsigned short accel_offset_y; /**< y acceleration offset */
    unsigned short accel_offset_z; /**< z acceleration offset */

    unsigned short mag_offset_x; /**< x magnetometer offset */
    unsigned short mag_offset_y; /**< y magnetometer offset */
    unsigned short mag_offset_z; /**< z magnetometer offset */

    unsigned short gyro_offset_x; /**< x gyroscrope offset */
    unsigned short gyro_offset_y; /**< y gyroscrope offset */
    unsigned short gyro_offset_z; /**< z gyroscrope offset */

    unsigned short accel_radius; /**< acceleration radius */

    unsigned short mag_radius; /**< magnetometer radius */
} offsetsBNO055;

/** Operation mode settings **/
typedef enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
} modeBNO055;
/*
+------------------+-------+---------+------+----------+----------+
| Mode             | Accel | Compass | Gyro | Fusion   | Fusion   |
|                  |       | (Mag)   |      | Absolute | Relative |
+==================+=======+=========+======+==========+==========+
| CONFIG_MODE      |   -   |   -     |  -   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| ACCONLY_MODE     |   X   |   -     |  -   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| MAGONLY_MODE     |   -   |   X     |  -   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| GYRONLY_MODE     |   -   |   -     |  X   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| ACCMAG_MODE      |   X   |   X     |  -   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| ACCGYRO_MODE     |   X   |   -     |  X   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| MAGGYRO_MODE     |   -   |   X     |  X   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| AMG_MODE         |   X   |   X     |  X   |     -    |     -    |
+------------------+-------+---------+------+----------+----------+
| IMUPLUS_MODE     |   X   |   -     |  X   |     -    |     X    |
+------------------+-------+---------+------+----------+----------+
| COMPASS_MODE     |   X   |   X     |  -   |     X    |     -    |
+------------------+-------+---------+------+----------+----------+
| M4G_MODE         |   X   |   X     |  -   |     -    |     X    |
+------------------+-------+---------+------+----------+----------+
| NDOF_FMC_OFF_MODE|   X   |   X     |  X   |     X    |     -    |
+------------------+-------+---------+------+----------+----------+
| NDOF_MODE        |   X   |   X     |  X   |     X    |     -    |
+------------------+-------+---------+------+----------+----------+
*/

typedef enum {
    GYRO_523HZ = 0x00, // For gyro_bandwidth property
    GYRO_230HZ = 0x08,
    GYRO_116HZ = 0x10,
    GYRO_47HZ = 0x18,
    GYRO_23HZ = 0x20,
    GYRO_12HZ = 0x28,
    GYRO_64HZ = 0x30,
    GYRO_32HZ = 0x38,  // Default
} GYRO_RATE;

typedef enum {
    GYRO_2000_DPS = 0x00, // Default. For gyro_range property
    GYRO_1000_DPS = 0x01,
    GYRO_500_DPS = 0x02,
    GYRO_250_DPS = 0x03,
    GYRO_125_DPS = 0x04,
} GYRO_DPS;


typedef enum {
    ACCEL_7_81HZ = 0x00, // For accel_bandwidth property
    ACCEL_15_63HZ = 0x08,
    ACCEL_62_5HZ = 0x0C,
    ACCEL_125HZ = 0x10,
    ACCEL_250HZ = 0x14,
    ACCEL_500HZ = 0x18,
    ACCEL_1000HZ = 0x1C
} ACC_RATE;

typedef enum {
    ACCEL_2G = 0x00, // Default. For accel_range property
    ACCEL_4G = 0x01,
    ACCEL_8G = 0x02,
    ACCEL_16G = 0x03
} ACC_G;

typedef enum {
    MAGNET_2HZ = 0x00,
    MAGNET_6HZ = 0x01,
    MAGNET_8HZ = 0x02,
    MAGNET_10HZ = 0x03,
    MAGNET_15HZ = 0x04,
    MAGNET_20HZ = 0x05, // Default
    MAGNET_25HZ = 0x06,
    MAGNET_30HZ = 0x07
} MAG_RATE;


typedef enum {
    /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0X07,

    /* Config register */
    BNO055_ACCEL_CONFIG_ADDR = 0x08,
    BNO055_GYRO_0_CONFIG_ADDR = 0x0A,
    BNO055_GYRO_1_CONFIG_ADDR = 0x0B,
    BNO055_MAG_CONFIG_ADDR = 0x09,

    /* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR = 0X1A,
    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

    /* Temperature data register */
    BNO055_TEMP_ADDR = 0X34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR = 0X35,
    BNO055_SELFTEST_RESULT_ADDR = 0X36,
    BNO055_INTR_STAT_ADDR = 0X37,

    BNO055_SYS_CLK_STAT_ADDR = 0X38,
    BNO055_SYS_STAT_ADDR = 0X39,
    BNO055_SYS_ERR_ADDR = 0X3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR = 0X3B,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR = 0X5B,
    MAG_OFFSET_X_MSB_ADDR = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR = 0X60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR = 0X61,
    GYRO_OFFSET_X_MSB_ADDR = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR = 0X66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0X67,
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A
} Register;


/** BNO055 power settings */
typedef enum {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
} Powermode_BNO055;

/** Remap settings **/
typedef enum {
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
} adafruit_bno055_axis_remap_config_t;

/** Remap Signs **/
typedef enum {
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
} adafruit_bno055_axis_remap_sign_t;

/** A structure to represent revisions **/
typedef struct {
    unsigned char accel_rev; /**< acceleration rev */
    unsigned char mag_rev;   /**< magnetometer rev */
    unsigned char gyro_rev;  /**< gyroscrope rev */
    unsigned short sw_rev;   /**< SW rev */
    unsigned char bl_rev;    /**< bootloader rev */
} adafruit_bno055_rev_info_t;

/** Vector Mappings **/
typedef enum {
    VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
    VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
    VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
    VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
} adafruit_vector_type_t;

#endif //OUTDOORNAV_ADAFRUIT_BNO055_CONSTANT_H
