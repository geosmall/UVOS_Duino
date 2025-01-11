#include "dev/icm42688p.h"
#include "uvos_brd.h"
#include <cstring>

#include "per/util/spi_util.h"

#include "icm_util.h"

static void Error_Handler()
{
    asm("bkpt 255");
    while (1)
    {
    }
}

// Uncomment to print ACCEL data (gyro is default)
// #define PRINT_ACCEL

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

#if defined(ARDUINO_FC_MatekH743)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
    constexpr Pin TX_PIN = Pin(PORTA, 9);
    constexpr Pin RX_PIN = Pin(PORTA, 10);
#elif defined(ARDUINO_NUCLEO_H753ZI)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_3;
    constexpr Pin TX_PIN = Pin(PORTD, 8);
    constexpr Pin RX_PIN = Pin(PORTD, 9);
#else // defined(DevEBoxH743VI)
    constexpr Pin CS_PIN = Pin(PORTA, 4);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTA, 7);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
    constexpr Pin TX_PIN = Pin(PORTA, 9);
    constexpr Pin RX_PIN = Pin(PORTA, 10);
#endif /* ARDUINO_FC_MatekH743 */

// Declare a UVOSboard object called hw
UVOSboard hw;
UartHandler uart;

// ICM42688P imu CS pin (using software driven CS)
GPIO csPin;

// Create INav busDevice, gyro objects
busDevice_t spi_dev;
gyroDev_t gyroDev;

// ICM42688P imu
ICM42688 imu;

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

    // Configure the Uart Peripheral to print out results
    UartHandler::Config uart_conf;
    uart_conf.periph        = UART_NUM;
    uart_conf.mode          = UartHandler::Config::Mode::TX;
    uart_conf.pin_config.tx = TX_PIN;
    uart_conf.pin_config.rx = RX_PIN;

    // Initialize the uart peripheral and start the DMA transmit
    uart.Init(uart_conf);

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

    // Calculate an SPI disable delay for configured SPI instance
    uint32_t calc_disable_delay = spi_compute_disable_delay_us(SpiInstancePtr_);

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

    spiIcmBusInit(&spi_dev);

    // Detect sensor
    bool isSensor = icm42605DeviceDetect(&spi_dev);
    if (!isSensor)
    {
        Error_Handler();
    }

    memset(&gyroDev, 0, sizeof(gyroDev));
    gyroDev = {
        .busDev = &spi_dev,
        .scale = (1.0f / 16.4f),     // 16.4 dps/lsb scalefactor
        .lpf = 0,
        .requestedSampleIntervalUs = 250,
        .dataReady = false,
        .sampleRateIntervalUs = 250,
        .gyroAlign = CW0_DEG_FLIP
    };

    // icm42605AccAndGyroInit(&gyroDev);
    if (icm42688Init(&gyroDev) != true)
    {
        Error_Handler();
    }


    // Initialize the ICM-42688P IMU
    // if (imu.Init(spi_handle) != ICM42688::Result::OK)
    // {
    //     Error_Handler();
    // }

    // Get IMU error to zero accelerometer and gyro readings, assuming sensor is level when powered up
    Calculate_IMU_Error();

    char buf[128];
// #if defined(PRINT_ACCEL)
//     int str_len = sprintf(buf, "ax,ay,az\r\n");
// #else
//     int str_len = sprintf(buf, "gx,gy,gz\r\n");
// #endif /* defined(PRINT_ACCEL) */
    int str_len = sprintf(buf, "ax,ay,az,gx,gy,gz\n");

    uart.BlockingTransmit((uint8_t*)buf, str_len);

    // Loop forever
    for (;;)
    {
        // Set the onboard LED
        hw.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Read the sensor
        // getIMUdata(); // Pulls raw gyro, accel data from IMU and LP filter to remove noise
        getAGT(&spi_dev);

        // Format the data into the buffer
// #if defined(PRINT_ACCEL)
//         int prn_buflen = snprintf(buf,
//                                   sizeof(buf),
//                                   "%.6f\t%.6f\t%.6f\n",
//                                   AccX,
//                                   AccY,
//                                   AccZ);
// #else
//         int prn_buflen = snprintf(buf,
//                                   sizeof(buf),
//                                   "%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n",
//                                   GyroX,
//                                   GyroY,
//                                   GyroZ,
//                                   -2000.0,
//                                   2000.0);
// #endif /* defined(PRINT_ACCEL) */

        int prn_buflen = snprintf(buf,
                                  sizeof(buf),
                                  "%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n",
                                  accX(),
                                  accY(),
                                  accZ(),
                                  gyrX(),
                                  gyrY(),
                                  gyrZ());

        // Print out formatted data
        uart.BlockingTransmit((uint8_t*) buf, prn_buflen);


        // Wait 500ms
        System::Delay(100);
    }
}

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