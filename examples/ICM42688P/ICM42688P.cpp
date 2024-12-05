#include "dev/icm42688p.h"
#include "uvos_brd.h"
#include <cstring>

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

// Declare a UVOSboard object called hw
UVOSboard hw;

// Handle we'll use to interact with IMU SPI
SpiHandle spi_handle;

// Structure to configure the IMU SPI instance
SpiHandle::Config spi_conf;

// ICM42688P imu
ICM42688 imu;

int main(void)
{
    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the Daisy Seed
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    // Configure the ICM-42688P IMU SPI interface (match for Matek_H743 WLITE)
    spi_conf.periph = SpiHandle::Config::Peripheral::SPI_1;
    spi_conf.mode = SpiHandle::Config::Mode::MASTER;
    spi_conf.direction = SpiHandle::Config::Direction::TWO_LINES;
    spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    spi_conf.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;

#ifdef USE_SOFT_NSS
    spi_conf.nss = SpiHandle::Config::NSS::SOFT;
#else
    spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
#endif /* USE_SOFT_NSS */

#ifdef ARDUINO_FC_MatekH743
    spi_conf.pin_config.nss = Pin(PORTC, 15);
    spi_conf.pin_config.sclk = Pin(PORTA, 5);
    spi_conf.pin_config.miso = Pin(PORTA, 6);
    spi_conf.pin_config.mosi = Pin(PORTD, 7);
#else
    spi_conf.pin_config.nss = Pin(PORTA, 4);
    spi_conf.pin_config.sclk = Pin(PORTA, 5);
    spi_conf.pin_config.miso = Pin(PORTA, 6);
    spi_conf.pin_config.mosi = Pin(PORTA, 7);
#endif /* ARDUINO_FC_MatekH743 */

    // spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_32;
    spi_handle.GetBaudHz(spi_conf.periph, DESIRED_SPI_FREQ, spi_conf.baud_prescaler);

    // Initialize the IMU SPI instance
    spi_handle.Init(spi_conf);

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