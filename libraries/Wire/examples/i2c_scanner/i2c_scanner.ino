#include "uvos_brd.h"
#include <Wire.h>

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

static constexpr size_t DMA_BUF_SIZE = 256;   // DMA rx buffer size
static uint8_t DMA_BUFFER_MEM_SECTION HardwareSerial_rx_buf[DMA_BUF_SIZE] = {0};

// Build full HardwareSerial::Config with nested lambda
static const HardwareSerial::Config serial_cfg = [] {
    HardwareSerial::Config c{};
    // UART settings
    c.uart_config.periph        = UartHandler::Config::Peripheral::USART_3;
    c.uart_config.mode          = UartHandler::Config::Mode::TX_RX;
    c.uart_config.pin_config.tx = Pin(PORTD, 8);
    c.uart_config.pin_config.rx = Pin(PORTD, 9);
    // DMA buffer
    c.dma_buf      = HardwareSerial_rx_buf;
    c.dma_buf_size = DMA_BUF_SIZE;
    return c;
}();

// Wrapper instance
HardwareSerial Serial(serial_cfg);

// Create LED objects
GPIO led0;
Pin led0_pin = Pin(PORTE, 1);

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    // Initialize led0 as an OUTPUT
    led0.Init(led0_pin, GPIO::Mode::OUTPUT);

    Serial.begin(115200);
    Wire.begin();
    Serial.println("I2C Scanner test...\r\n\r\n");

    byte error, address;
    int nDevices;

    Serial.println("Starting I2C Scanning: \r\n");

    // Use a GY-85 sensor board for scan test, which has:
    //   ADXL345 -  0x53 — Three axis acceleration 
    //   ITG3205  - 0x68 — Three axis gyroscope (AD0 is GND)
    //   HMC5883L - 0x1E — Three axis magnetic field

    // Scan thru range of valid 7-bit slave addresses.
    nDevices = 0;

    // Valid slave addresses are >= than 0x08 and <= than 0x77.
    for (address = 0x08; address <= 0x77; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);

            nDevices++;
        } else if (error == 4) {
            Serial.print("Return error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    } else {
        Serial.println("done");
    }

    // Scan complete, loop forever
    while (true) {
        // Toggle the LED state for the next time around.
        led0.Toggle();

        // Delay for a bit
        System::Delay(500);
    }
}

/*
Gives:

I2C Scanner test...


Starting I2C Scanning: 

Return error at address 0x08
Return error at address 0x09
Return error at address 0x0A
Return error at address 0x0B
Return error at address 0x0C
Return error at address 0x0D
Return error at address 0x0E
Return error at address 0x0F
Return error at address 0x10
Return error at address 0x11
Return error at address 0x12
Return error at address 0x13
Return error at address 0x14
Return error at address 0x15
Return error at address 0x16
Return error at address 0x17
Return error at address 0x18
Return error at address 0x19
Return error at address 0x1A
Return error at address 0x1B
Return error at address 0x1C
Return error at address 0x1D
I2C device found at address 0x1E
Return error at address 0x1F
Return error at address 0x20
Return error at address 0x21
Return error at address 0x22
Return error at address 0x23
Return error at address 0x24
Return error at address 0x25
Return error at address 0x26
Return error at address 0x27
Return error at address 0x28
Return error at address 0x29
Return error at address 0x2A
Return error at address 0x2B
Return error at address 0x2C
Return error at address 0x2D
Return error at address 0x2E
Return error at address 0x2F
Return error at address 0x30
Return error at address 0x31
Return error at address 0x32
Return error at address 0x33
Return error at address 0x34
Return error at address 0x35
Return error at address 0x36
Return error at address 0x37
Return error at address 0x38
Return error at address 0x39
Return error at address 0x3A
Return error at address 0x3B
Return error at address 0x3C
Return error at address 0x3D
Return error at address 0x3E
Return error at address 0x3F
Return error at address 0x40
Return error at address 0x41
Return error at address 0x42
Return error at address 0x43
Return error at address 0x44
Return error at address 0x45
Return error at address 0x46
Return error at address 0x47
Return error at address 0x48
Return error at address 0x49
Return error at address 0x4A
Return error at address 0x4B
Return error at address 0x4C
Return error at address 0x4D
Return error at address 0x4E
Return error at address 0x4F
Return error at address 0x50
Return error at address 0x51
Return error at address 0x52
I2C device found at address 0x53
Return error at address 0x54
Return error at address 0x55
Return error at address 0x56
Return error at address 0x57
Return error at address 0x58
Return error at address 0x59
Return error at address 0x5A
Return error at address 0x5B
Return error at address 0x5C
Return error at address 0x5D
Return error at address 0x5E
Return error at address 0x5F
Return error at address 0x60
Return error at address 0x61
Return error at address 0x62
Return error at address 0x63
Return error at address 0x64
Return error at address 0x65
Return error at address 0x66
Return error at address 0x67
I2C device found at address 0x68
Return error at address 0x69
Return error at address 0x6A
Return error at address 0x6B
Return error at address 0x6C
Return error at address 0x6D
Return error at address 0x6E
Return error at address 0x6F
Return error at address 0x70
Return error at address 0x71
Return error at address 0x72
Return error at address 0x73
Return error at address 0x74
Return error at address 0x75
Return error at address 0x76
Return error at address 0x77
done

*/