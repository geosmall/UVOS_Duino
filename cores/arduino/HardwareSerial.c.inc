/****************************************************************
 * serial_demo.cpp – Minimal test for HardwareSerial.hpp
 * Hardware: STM32H743  (USART1 on PA9/PA10, AF7)
 ****************************************************************/
#include "uvos_brd.h"
#include "HardwareSerial.hpp"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

/*----------------------------------------------------------------
 * 1)  Board‑specific pin descriptors
 *----------------------------------------------------------------
 * uvs_gpio_pin is whatever struct your driver already uses.
 * Only the fields actually accessed by the low‑level driver
 * need to be filled in here.
 */
// static const uvs_gpio_pin TX1 = Pin(PORTD, 8);
// static const uvs_gpio_pin RX1 = Pin(PORTD, 9);

/*----------------------------------------------------------------
 * 2)  Instantiate a HardwareSerial object
 *     Template parameters are  <RxRing, TxRing, DmaChunk>.
 *     Defaults (1024, 512, 256) are fine for most use‑cases.
 *----------------------------------------------------------------*/
uvos::HardwareSerial Serial3(
    uvos::UartHandler::Config::Peripheral::USART_3,
    Pin(PORTD, 8),  // TX pin
    Pin(PORTD, 9)); // RX pin

/*----------------------------------------------------------------
 * 3)  Application
 *----------------------------------------------------------------*/
static void send_banner()
{
    Serial3.println("\r\n=== HardwareSerial DMA demo ===");
}

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    Serial3.begin(115200); // default framing SERIAL_8N1
    // while (!Serial3) {
    //     ; // wait for serial port to connect. Needed for native USB port only
    // }

    // send_banner();
    Serial3.println("\r\n=== HardwareSerial DMA demo ===");

    uint32_t last = 0;
    while (true)
    {
        /* Echo any incoming data */
        while (Serial3.available())
            Serial3.write(Serial3.read());

        /* Send a heartbeat every second (simple tick counter) */
        if (System::GetTick() - last >= 1000)    // GetTick = HAL tick (ms)
        {
            last = System::GetTick();
            Serial3.print("free TX slots: ");
            Serial3.println(Serial3.availableForWrite());
        }
    }
}
