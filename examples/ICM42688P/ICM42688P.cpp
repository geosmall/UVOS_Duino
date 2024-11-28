#include "dev/icm42688p.h"
#include "uvos_brd.h"
#include <string.h>

static void Error_Handler()
{
    asm("bkpt 255");
    while (1)
    {
    }
}

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

uint8_t sumbuff[1024];

void UsbCallback(uint8_t* buf, uint32_t* len)
{
    for (size_t i = 0; i < *len; i++)
    {
        sumbuff[i] = buf[i];
    }
}

int main(void)
{
    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the board with USB
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();
    hw.usb_handle.Init(UsbHandle::FS_INTERNAL);

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

    // ICM42688P imu
    ICM42688 imu(spi_handle, 2'000'000);

    // start communication with IMU
    int status = imu.begin();
    if (status < 0)
    {
        Error_Handler();
    }

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
        imu.getAGT();

        // Clear the buffer
        memset(outputBuffer, 0, sizeof(outputBuffer));

        // Format the data into the buffer
        int prn_buflen = snprintf(outputBuffer,
                                  sizeof(outputBuffer),
                                  "%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n",
                                  imu.accX(),
                                  imu.accY(),
                                  imu.accZ(),
                                  imu.gyrX(),
                                  imu.gyrY(),
                                  imu.gyrZ());
        hw.usb_handle.TransmitInternal((uint8_t*) outputBuffer, prn_buflen);

        // Wait 500ms
        System::Delay(100);
    }
}
