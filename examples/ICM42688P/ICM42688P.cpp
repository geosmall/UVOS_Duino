#include "dev/icm42688p.h"
#include "uvos_brd.h"
#include <cstring>

#include "per/util/spi_util.h"
#include "stm32h7xx_ll_spi.h"

static void Error_Handler()
{
    asm("bkpt 255");
    while (1)
    {
    }
}

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // Default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

#if defined(GYRO_250DPS)
#define GYRO_SCALE ICM42688::GyroFS::dps250
#define GYRO_SCALE_FACTOR 131.0
#elif defined(GYRO_500DPS)
#define GYRO_SCALE ICM42688::GyroFS::dps500
#define GYRO_SCALE_FACTOR 65.5
#elif defined(GYRO_1000DPS)
#define GYRO_SCALE ICM42688::GyroFS::dps1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined(GYRO_2000DPS)
#define GYRO_SCALE ICM42688::GyroFS::dps2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined(ACCEL_2G)
#define ACCEL_SCALE ICM42688::AccelFS::gpm2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined(ACCEL_4G)
#define ACCEL_SCALE ICM42688::AccelFS::gpm4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined(ACCEL_8G)
#define ACCEL_SCALE ICM42688::AccelFS::gpm8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined(ACCEL_16G)
#define ACCEL_SCALE ICM42688::AccelFS::gpm16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

// IMU global data:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;

// Filter parameters - Defaults tuned for 2kHz loop rate
float B_madgwick = 0.04; // Madgwick filter parameter
float B_accel = 0.14;    // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;      // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;       // Magnetometer LP filter parameter

// IMU calibration parameters
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;

// Forward declare helper functions
void Calculate_IMU_Error();
void getIMUdata();

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Uncomment to use software driven NSS
#define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1000000

#if defined(ARDUINO_FC_MatekH743) || defined(ARDUINO_NUCLEO_H753ZI)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
#else
    constexpr Pin CS_PIN = Pin(PORTA, 4);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTA, 7);
#endif /* ARDUINO_FC_MatekH743 */

// Declare a UVOSboard object called hw
UVOSboard hw;

// ICM42688P imu CS pin (using software driven CS)
GPIO csPin;

typedef struct busDevice_s {
    uint32_t flags;             // Copy of flags
    struct {
        SpiHandle spiBus;       // SPI bus ID
        uint32_t disable_delay;
        GPIO csnPin;            // IO for CS# pin
    } spi;
} busDevice_t;

busDevice_t spi_dev;

// ICM42688P imu
ICM42688 imu;

static bool icm42605DeviceDetect(busDevice_t* dev);

int main(void)
{
    SpiHandle spi_handle;         // Handle we'll use to interact with IMU SPI
    SpiHandle::Config spi_conf;   // Structure to configure the IMU SPI instance
    SPI_TypeDef* SpiInstancePtr_; // Pointer to SPI instance

    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    // Give ICM-42688P some time to stabilize
    System::Delay(5);

    // Configure the ICM-42688P IMU SPI interface (match for Matek_H743 WLITE)
    spi_conf.periph = SpiHandle::Config::Peripheral::SPI_1;
    spi_conf.mode = SpiHandle::Config::Mode::MASTER;
    spi_conf.direction = SpiHandle::Config::Direction::TWO_LINES;
    spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    spi_conf.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;

    // Save away the SPI instance for later use
    SpiInstancePtr_ = SpiHandle::PeripheralToHAL(spi_conf.periph);
    uint32_t calc_disable_delay = spi_compute_disable_delay_us(SpiInstancePtr_);

#ifdef USE_SOFT_NSS
    spi_conf.nss = SpiHandle::Config::NSS::SOFT;
#else
    spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
#endif /* USE_SOFT_NSS */

    spi_conf.pin_config.nss = CS_PIN;
    spi_conf.pin_config.sclk = SCLK_PIN;
    spi_conf.pin_config.miso = MISO_PIN;
    spi_conf.pin_config.mosi = MOSI_PIN;

    // spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_32;
    spi_handle.GetBaudHz(spi_conf.periph, DESIRED_SPI_FREQ, spi_conf.baud_prescaler);

    // Initialize the IMU SPI instance
    spi_handle.Init(spi_conf);

    // Initialize chip select (CS) pin
    // csPin.Init(Pin(PORTC, 15), GPIO::Mode::OUTPUT, GPIO::Pull::PULLUP);
    csPin.Init(CS_PIN, GPIO::Mode::OUTPUT, GPIO::Pull::PULLUP);

    // Instance of the bus device, Initialize with default values
    spi_dev = {
        .flags = 0,
        .spi = {
            .spiBus = spi_handle,
            .disable_delay = calc_disable_delay,
            .csnPin = csPin
        }
    };

    // Detect sensor
    bool isSensor = icm42605DeviceDetect(&spi_dev);
    if (!isSensor)
    {
        Error_Handler();
    }

    // Initialize the ICM-42688P IMU
    if (imu.Init(spi_handle) != ICM42688::Result::OK)
    {
        Error_Handler();
    }

    // Get IMU error to zero accelerometer and gyro readings, assuming sensor is level when powered up
    Calculate_IMU_Error();

    char buff[512];
    sprintf(buff, "ax,ay,az,gx,gy,gz\r\n");
    hw.usb_handle.TransmitInternal((uint8_t*) buff, strlen(buff));

    char outputBuffer[128]; // Buffer for the entire formatted string

    // Loop forever
    for (;;)
    {
        // Set the onboard LED
        hw.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Read the sensor
        getIMUdata(); // Pulls raw gyro, accel data from IMU and LP filter to remove noise

        // Format the data into the buffer
        int prn_buflen = snprintf(outputBuffer,
                                  sizeof(outputBuffer),
                                  "%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n",
                                  AccX,
                                  AccY,
                                  AccZ,
                                  GyroX,
                                  GyroY,
                                  GyroZ);
        hw.usb_handle.TransmitInternal((uint8_t*) outputBuffer, prn_buflen);

        // Wait 500ms
        System::Delay(100);
    }
}

/* ============== spi.c =================*/

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

#define ICM42605_RA_PWR_MGMT0       0x4E

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

#define MPU_RA_WHO_AM_I             0x75

#define ICM42605_WHO_AM_I_CONST    (0x42)
#define ICM42688P_WHO_AM_I_CONST   (0x47)

static bool is42688P = false;

void spiChipSelectSetupDelay(void)
{
    // CS->CLK delay, MPU6000 - 8ns
    // CS->CLK delay, ICM42688P - 39ns
    System::DelayNs(39);
}

void spiChipSelectHoldTime(void)
{
    // CLK->CS delay, MPU6000 - 500ns
    // CS->CLK delay, ICM42688P - 18ns
    System::DelayNs(18);
}

void spiBusSelectDevice(const busDevice_t* dev)
{
    // IOLo(dev->busdev.spi.csnPin);
    csPin.Write(GPIO_PIN_RESET);
    // const_cast<uvos::GPIO&>(dev->spi.csnPin).Write(GPIO_PIN_RESET);
    spiChipSelectSetupDelay();
}

void spiBusDeselectDevice(const busDevice_t* dev)
{
    spiChipSelectHoldTime();
    // IOHi(dev->busdev.spi.csnPin);
    csPin.Write(GPIO_PIN_SET);
    // const_cast<uvos::GPIO&>(dev->spi.csnPin).Write(GPIO_PIN_SET);
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *rxData, const uint8_t *txData, int len)
{
    LL_SPI_SetTransferSize(instance, len);
    LL_SPI_Enable(instance);
    LL_SPI_StartMasterTransfer(instance);
    while (len) {
        int spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_TXP(instance)) {
            if ((spiTimeout--) == 0) {
                // spiTimeoutUserCallback(instance); gls
                return false;
            }
        }
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);

        spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_RXP(instance)) {
            if ((spiTimeout--) == 0) {
                // spiTimeoutUserCallback(instance); gls
                return false;
            }
        }
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }
    while (!LL_SPI_IsActiveFlag_EOT(instance));
    // Add a delay before disabling SPI otherwise last-bit/last-clock may be truncated
    // See https://github.com/stm32duino/Arduino_Core_STM32/issues/1294
    // Computed delay is half SPI clock
    System::DelayUs(spi_dev.spi.disable_delay);

    LL_SPI_ClearFlag_TXTF(instance);
    LL_SPI_Disable(instance);

    return true;
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte)
{
    uint8_t value = 0xFF;
    if (!spiTransfer(instance, &value, &txByte, 1)) {
        return 0xFF;
    }
    return value;
}

void busSetSpeed(const busDevice_t* dev, busSpeed_e speed)
{

}

bool spiBusWriteRegister(const busDevice_t* dev, uint8_t reg, uint8_t data)
{
    SpiHandle::Config cfg = dev->spi.spiBus.GetConfig();
    SPI_TypeDef* instance = SpiHandle::PeripheralToHAL(cfg.periph);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusSelectDevice(dev);
    }

    spiTransferByte(instance, reg);
    spiTransferByte(instance, data);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusDeselectDevice(dev);
    }

    return true;
}

bool spiBusReadRegister(const busDevice_t* dev, uint8_t reg, uint8_t* data)
{
    SpiHandle::Config cfg = dev->spi.spiBus.GetConfig();
    SPI_TypeDef* instance = SpiHandle::PeripheralToHAL(cfg.periph);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusSelectDevice(dev);
    }

    spiTransferByte(instance, reg);
    spiTransfer(instance, data, NULL, 1);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusDeselectDevice(dev);
    }

    return true;
}

bool busWrite(const busDevice_t* dev, uint8_t reg, uint8_t data)
{
    if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
        return spiBusWriteRegister(dev, reg, data);
    } else {
        return spiBusWriteRegister(dev, reg & 0x7F, data);
    }
}

bool busRead(const busDevice_t* dev, uint8_t reg, uint8_t* data)
{
    if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
        return spiBusReadRegister(dev, reg, data);
    } else {
        return spiBusReadRegister(dev, reg | 0x80, data);
    }
}

static void setUserBank(const busDevice_t *dev, const uint8_t user_bank)
{
    busWrite(dev, ICM426XX_RA_REG_BANK_SEL, user_bank & 7);
}

static bool icm42605DeviceDetect(busDevice_t* dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    busWrite(dev, ICM42605_RA_PWR_MGMT0, 0x00);

    do {
        System::Delay(150);

        busRead(dev, MPU_RA_WHO_AM_I, &tmp);

        switch (tmp) {
        /* ICM42605 and ICM42688P share the register structure*/
        case ICM42605_WHO_AM_I_CONST:
            is42688P = false;
            return true;
        case ICM42688P_WHO_AM_I_CONST:
            is42688P = true;
            return true;

        default:
            // Retry detection
            break;
        }
    } while (attemptsRemaining--);

    return false;
}

/* ============== spi.c =================*/

void Calculate_IMU_Error()
{
    // DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on
    // flat surface

    int16_t AcX = 0, AcY = 0, AcZ = 0, GyX = 0, GyY = 0, GyZ = 0;
    AccErrorX = 0.0;
    AccErrorY = 0.0;
    AccErrorZ = 0.0;
    GyroErrorX = 0.0;
    GyroErrorY = 0.0;
    GyroErrorZ = 0.0;

    // Read IMU values 12000 times
    int c = 0;
    while (c < 12000)
    {
        imu.getIMU6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

        AccX = AcX / ACCEL_SCALE_FACTOR;
        AccY = AcY / ACCEL_SCALE_FACTOR;
        AccZ = AcZ / ACCEL_SCALE_FACTOR;
        GyroX = GyX / GYRO_SCALE_FACTOR;
        GyroY = GyY / GYRO_SCALE_FACTOR;
        GyroZ = GyZ / GYRO_SCALE_FACTOR;

        // Sum all readings
        AccErrorX = AccErrorX + AccX;
        AccErrorY = AccErrorY + AccY;
        AccErrorZ = AccErrorZ + AccZ;
        GyroErrorX = GyroErrorX + GyroX;
        GyroErrorY = GyroErrorY + GyroY;
        GyroErrorZ = GyroErrorZ + GyroZ;
        c++;
    }

    // Divide the sum by 12000 to get the error value
    AccErrorX = AccErrorX / c;
    AccErrorY = AccErrorY / c;
    AccErrorZ = AccErrorZ / c - 1.0;
    GyroErrorX = GyroErrorX / c;
    GyroErrorY = GyroErrorY / c;
    GyroErrorZ = GyroErrorZ / c;
}

void getIMUdata()
{
    // DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
    /*
     * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ.
     * These values are scaled according to the IMU datasheet to put them into correct units of g's,
     * deg/sec. A simple first-order low-pass filter is used to get rid of high frequency noise in these
     * raw signals. Generally you want to cut off everything past 80Hz, but if your loop rate is not fast enough,
     * the low pass filter will cause a lag in the readings. The filter parameters B_gyro and B_accel are set to be
     * good for a 2kHz loop rate. Finally, the constant errors found in calculate_IMU_error() on startup are
     * subtracted from the accelerometer and gyro readings.
     */
    int16_t AcX = 0, AcY = 0, AcZ = 0, GyX = 0, GyY = 0, GyZ = 0;

    imu.getIMU6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    // Accelerometer
    AccX = AcX / ACCEL_SCALE_FACTOR; // G's
    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;
    // Correct the outputs with the calculated error values
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccZ = AccZ - AccErrorZ;
    // LP filter accelerometer data
    AccX = (1.0 - B_accel) * AccX_prev + B_accel * AccX;
    AccY = (1.0 - B_accel) * AccY_prev + B_accel * AccY;
    AccZ = (1.0 - B_accel) * AccZ_prev + B_accel * AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;

    // Gyro
    GyroX = GyX / GYRO_SCALE_FACTOR; // deg/sec
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    // Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
    // LP filter gyro data
    GyroX = (1.0 - B_gyro) * GyroX_prev + B_gyro * GyroX;
    GyroY = (1.0 - B_gyro) * GyroY_prev + B_gyro * GyroY;
    GyroZ = (1.0 - B_gyro) * GyroZ_prev + B_gyro * GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
}