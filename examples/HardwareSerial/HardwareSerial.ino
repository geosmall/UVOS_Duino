/****************************************************************
 * serial_demo.cpp – Minimal test for HardwareSerial.hpp
 * Hardware: STM32H743  (USART1 on PA9/PA10, AF7)
 ****************************************************************/
#include "uvos_brd.h"
#include "HardwareSerial.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

/* Fully initialise the config at static‑init time */
static const uvos::UartHandler::Config uart_cfg = []{
    uvos::UartHandler::Config c;
    c.periph        = uvos::UartHandler::Config::Peripheral::USART_3;
    c.mode          = uvos::UartHandler::Config::Mode::TX_RX;
    c.pin_config.tx = Pin(PORTD, 8);
    c.pin_config.rx = Pin(PORTD, 9);
    return c;
}();

/* Wrapper instance – safe because cfg is already complete */
uvos_arduino::HardwareSerial Serial3(uart_cfg);

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    Serial3.begin(115200);



    Serial3.println("STM32H743 echo test - type away!");

    uint32_t last = 0;
    while (true)
    {
        while(Serial3.available()) {
            Serial3.write(static_cast<uint8_t>(Serial3.read()));
        }
    }
}
