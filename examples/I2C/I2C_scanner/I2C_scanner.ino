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

// Create LED pins
Pin led0_pin = Pin(PORTE, 1);

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

    // Initialize led0 as an OUTPUT
    led0.Init(led0_pin, GPIO::Mode::OUTPUT);

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
    //   ITG3205  - 0x68 — Three axis gyroscope (AD0 is GND)
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

/* Gives:

I2C Scanner test...


Starting I2C Scanning: 

No I2C device at address: 0x08
No I2C device at address: 0x09
No I2C device at address: 0x0A
No I2C device at address: 0x0B
No I2C device at address: 0x0C
No I2C device at address: 0x0D
No I2C device at address: 0x0E
No I2C device at address: 0x0F
No I2C device at address: 0x10
No I2C device at address: 0x11
No I2C device at address: 0x12
No I2C device at address: 0x13
No I2C device at address: 0x14
No I2C device at address: 0x15
No I2C device at address: 0x16
No I2C device at address: 0x17
No I2C device at address: 0x18
No I2C device at address: 0x19
No I2C device at address: 0x1A
No I2C device at address: 0x1B
No I2C device at address: 0x1C
No I2C device at address: 0x1D
I2C device found >>> address: 0x1E
No I2C device at address: 0x1F
No I2C device at address: 0x20
No I2C device at address: 0x21
No I2C device at address: 0x22
No I2C device at address: 0x23
No I2C device at address: 0x24
No I2C device at address: 0x25
No I2C device at address: 0x26
No I2C device at address: 0x27
No I2C device at address: 0x28
No I2C device at address: 0x29
No I2C device at address: 0x2A
No I2C device at address: 0x2B
No I2C device at address: 0x2C
No I2C device at address: 0x2D
No I2C device at address: 0x2E
No I2C device at address: 0x2F
No I2C device at address: 0x30
No I2C device at address: 0x31
No I2C device at address: 0x32
No I2C device at address: 0x33
No I2C device at address: 0x34
No I2C device at address: 0x35
No I2C device at address: 0x36
No I2C device at address: 0x37
No I2C device at address: 0x38
No I2C device at address: 0x39
No I2C device at address: 0x3A
No I2C device at address: 0x3B
No I2C device at address: 0x3C
No I2C device at address: 0x3D
No I2C device at address: 0x3E
No I2C device at address: 0x3F
No I2C device at address: 0x40
No I2C device at address: 0x41
No I2C device at address: 0x42
No I2C device at address: 0x43
No I2C device at address: 0x44
No I2C device at address: 0x45
No I2C device at address: 0x46
No I2C device at address: 0x47
No I2C device at address: 0x48
No I2C device at address: 0x49
No I2C device at address: 0x4A
No I2C device at address: 0x4B
No I2C device at address: 0x4C
No I2C device at address: 0x4D
No I2C device at address: 0x4E
No I2C device at address: 0x4F
No I2C device at address: 0x50
No I2C device at address: 0x51
No I2C device at address: 0x52
I2C device found >>> address: 0x53
No I2C device at address: 0x54
No I2C device at address: 0x55
No I2C device at address: 0x56
No I2C device at address: 0x57
No I2C device at address: 0x58
No I2C device at address: 0x59
No I2C device at address: 0x5A
No I2C device at address: 0x5B
No I2C device at address: 0x5C
No I2C device at address: 0x5D
No I2C device at address: 0x5E
No I2C device at address: 0x5F
No I2C device at address: 0x60
No I2C device at address: 0x61
No I2C device at address: 0x62
No I2C device at address: 0x63
No I2C device at address: 0x64
No I2C device at address: 0x65
No I2C device at address: 0x66
No I2C device at address: 0x67
I2C device found >>> address: 0x68
No I2C device at address: 0x69
No I2C device at address: 0x6A
No I2C device at address: 0x6B
No I2C device at address: 0x6C
No I2C device at address: 0x6D
No I2C device at address: 0x6E
No I2C device at address: 0x6F
No I2C device at address: 0x70
No I2C device at address: 0x71
No I2C device at address: 0x72
No I2C device at address: 0x73
No I2C device at address: 0x74
No I2C device at address: 0x75
No I2C device at address: 0x76
No I2C device at address: 0x77

*/