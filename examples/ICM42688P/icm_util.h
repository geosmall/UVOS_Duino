#pragma once

#include "uvos.h"

/* ------- maths.h --------- */

#define _CHOOSE2(binoper, lexpr, lvar, rexpr, rvar) \
    ( __extension__ ({                              \
        __typeof__(lexpr) lvar = (lexpr);           \
        __typeof__(rexpr) rvar = (rexpr);           \
        lvar binoper rvar ? lvar : rvar;            \
    }))
#define _CHOOSE_VAR2(prefix, unique) prefix##unique
#define _CHOOSE_VAR(prefix, unique) _CHOOSE_VAR2(prefix, unique)
#define _CHOOSE(binoper, lexpr, rexpr)          \
    _CHOOSE2(                                   \
        binoper,                                \
        lexpr, _CHOOSE_VAR(_left, __COUNTER__), \
        rexpr, _CHOOSE_VAR(_right, __COUNTER__) \
        )
#define MIN(a, b) _CHOOSE(<, a, b)
#define MAX(a, b) _CHOOSE(>, a, b)
#define SIGN(a) ((a >= 0) ? 1 : -1)

#define _ABS_II(x, var)             \
    ( __extension__ ({              \
        __typeof__(x) var = (x);    \
        var < 0 ? -var : var;       \
    }))
#define _ABS_I(x, var) _ABS_II(x, var)
#define ABS(x) _ABS_I(x, _CHOOSE_VAR(_abs, __COUNTER__))

/* ------ config.h ------- */

#define TASK_GYRO_LOOPTIME 500 // Task gyro always runs at 2kHz = 500 uSec

/* ------- axis.h --------- */

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

#define XYZ_AXIS_COUNT 3

/* ------- bus.h --------- */

typedef struct busDevice_s {
    uint32_t flags;             // Copy of flags
    struct {
        uvos::SpiHandle spiBus;       // SPI bus ID
        uint32_t disable_delay;
        uvos::GPIO csnPin;            // IO for CS# pin
    } spi;
} busDevice_t;

typedef enum {
    DEVFLAGS_NONE                       = 0,
    DEVFLAGS_USE_RAW_REGISTERS          = (1 << 0),     // Don't manipulate MSB for R/W selection (SPI), allow using 0xFF register to raw i2c reads/writes

    // SPI-only
    DEVFLAGS_USE_MANUAL_DEVICE_SELECT   = (1 << 1),     // (SPI only) Don't automatically select/deselect device
    DEVFLAGS_SPI_MODE_0                 = (1 << 2),     // (SPI only) Use CPOL=0/CPHA=0 (if unset MODE3 is used - CPOL=1/CPHA=1)
} deviceFlags_e;

typedef enum {
    BUS_SPEED_INITIALIZATION = 0,
    BUS_SPEED_SLOW           = 1,
    BUS_SPEED_STANDARD       = 2,
    BUS_SPEED_FAST           = 3,
    BUS_SPEED_ULTRAFAST      = 4
} busSpeed_e;

/* ------ sensor.h ------- */

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

/* ------- gyro.h --------- */

typedef struct gyro_s {
    bool initialized;
    uint32_t targetLooptime;
    float gyroADCf[XYZ_AXIS_COUNT];
    float gyroRaw[XYZ_AXIS_COUNT];
} gyro_t;

/* ------ accgyro.h ------- */

#define GYRO_LPF_256HZ      0
#define GYRO_LPF_188HZ      1
#define GYRO_LPF_98HZ       2
#define GYRO_LPF_42HZ       3
#define GYRO_LPF_20HZ       4
#define GYRO_LPF_10HZ       5
#define GYRO_LPF_5HZ        6
#define GYRO_LPF_NONE       7

typedef struct {
    uint8_t gyroLpf;
    uint16_t gyroRateHz;
    uint8_t gyroConfigValues[2];
} gyroFilterAndRateConfig_t;

typedef struct gyroDev_s {
    busDevice_t* busDev;
    // sensorGyroInitFuncPtr initFn;                       // initialize function
    // sensorGyroReadFuncPtr readFn;                       // read 3 axis data function
    // sensorGyroReadDataFuncPtr temperatureFn;            // read temperature if available
    // sensorGyroInterruptStatusFuncPtr intStatusFn;
    // sensorGyroUpdateFuncPtr updateFn;
    float scale;                                        // scalefactor
    float gyroADCRaw[XYZ_AXIS_COUNT];
    float gyroZero[XYZ_AXIS_COUNT];
    // uint8_t imuSensorToUse;
    uint8_t lpf;                                        // Configuration value: Hardware LPF setting
    uint32_t requestedSampleIntervalUs;                 // Requested sample interval
    volatile bool dataReady;
    uint32_t sampleRateIntervalUs;                      // Gyro driver should set this to actual sampling rate as signaled by IRQ
    sensor_align_e gyroAlign;
} gyroDev_t;

typedef struct accDev_s {
    busDevice_t * busDev;
    // sensorAccInitFuncPtr initFn;                        // initialize function
    // sensorAccReadFuncPtr readFn;                        // read 3 axis data function
    uint16_t acc_1G;
    float ADCRaw[XYZ_AXIS_COUNT];
    uint8_t imuSensorToUse;
    sensor_align_e accAlign;
} accDev_t;

void spiIcmBusInit(const busDevice_t* dev);
bool icm42605DeviceDetect(const busDevice_t* dev);
void icm42605AccAndGyroInit(gyroDev_t *gyro);
bool icm42688Init(gyroDev_t* gyro);
int getAGT(const busDevice_t *dev);

float accX();
float accY();
float accZ();

float gyrX();
float gyrY();
float gyrZ();

float temp();
