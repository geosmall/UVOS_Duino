#include "uvos_brd.h"
#include <cstring>

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;
UartHandler uart;

I2CHandle i2c_dev;
static constexpr I2CHandle::Config i2c_dev_config= {
    I2CHandle::Config::Peripheral::I2C_2,
    {
        {UVS_GPIOB, 10}, // SCL
        {UVS_GPIOB, 11}  // SDA
    },
    I2CHandle::Config::Speed::I2C_400KHZ
};

// Create LED objects
GPIO led0;
GPIO led1;

// Create LED pins
Pin led0_pin = Pin(PORTE, 3);
Pin led1_pin = Pin(PORTE, 4);

// Function to print a message over uart
void print_msg(const char* msg)
{
    char buf[128];
    sprintf(buf, "%s\r\n", msg);
    uart.BlockingTransmit((uint8_t*)buf, strlen(buf));
}

int main(void)
{
    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    // Configure the Uart Peripheral
    UartHandler::Config uart_conf;
    uart_conf.periph        = UartHandler::Config::Peripheral::USART_3;
    uart_conf.mode          = UartHandler::Config::Mode::TX;
    uart_conf.pin_config.tx = Pin(PORTD, 8);
    uart_conf.pin_config.rx = Pin(PORTD, 9);

    // Initialize the uart peripheral for test results
    uart.Init(uart_conf);

    print_msg("I2C Scanner test...\r\n\r\n");

    i2c_dev.Init(i2c_dev_config);

    print_msg("Starting I2C Scanning: \r\n");

    char prn_buf[128];

    // Use a GY-85 sensor board for scan test, which has:
    //   ADXL345 -  0x53 — Three axis acceleration 
    //   ITG3205  - 0x69 — Three axis gyroscope
    //   HMC5883L - 0x1E — Three axis magnetic field

    // Scan thru range of valid 7-bit slave addresses.
    // Valid slave addresses are >= than 0x08 and <= than 0x77.
    for(uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        // ST HAL expects 7-bit address passed as left aligned (shift left 1 bit)
        uint16_t HAL_addr = ((uint16_t)addr) << 1;

        if (i2c_dev.IsDeviceReady(HAL_addr, 2, 5) == I2CHandle::Result::OK) {
            sprintf(prn_buf, "I2C device found >>> address: 0x%02X\r\n", addr);
            uart.BlockingTransmit((uint8_t*)prn_buf, strlen(prn_buf));
        } else {
            sprintf(prn_buf, "No I2C device at address: 0x%02X\r\n", addr);
            uart.BlockingTransmit((uint8_t*)prn_buf, strlen(prn_buf));
        }
    }

    // Scan complete, loop forever
    while (true) {
        // Toggle the LED state for the next time around.
        led0.Toggle();

        // Delay for a bit
        System::Delay(100);
    }
}
