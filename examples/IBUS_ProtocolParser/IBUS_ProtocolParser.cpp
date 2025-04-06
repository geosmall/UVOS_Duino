#include "uvos.h"
#include "uvos_brd.h"
#include <cstring>

/** This prevents us from having to type "uvos::" in front of a lot of things. */
using namespace uvos;

// Uncomment to use software driven NSS
#define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1'000'000

#if defined(ARDUINO_FC_MatekH743) || defined(ARDUINO_NUCLEO_H753ZI)
  constexpr SpiHandle::Config::Peripheral IMU_SPI_NUM = SpiHandle::Config::Peripheral::SPI_1;
  constexpr Pin IMU_CS_PIN = Pin(PORTC, 15);
  constexpr Pin IMU_SCLK_PIN = Pin(PORTA, 5);
  constexpr Pin IMU_MISO_PIN = Pin(PORTA, 6);
  constexpr Pin IMU_MOSI_PIN = Pin(PORTD, 7);
  constexpr Pin IMU_INT1_PIN = Pin(PORTB, 2);
  constexpr UartHandler::Config::Peripheral DBG_UART_NUM = UartHandler::Config::Peripheral::USART_3;
  constexpr Pin DBG_TX_PIN = Pin(PORTD, 8);
  constexpr Pin DBG_RX_PIN = Pin(PORTD, 9);
  constexpr UartHandler::Config::Peripheral SER_RX_UART_NUM = UartHandler::Config::Peripheral::USART_6;
  constexpr Pin SER_RX_TX_PIN = Pin(PORTC, 6);
  constexpr Pin SER_RX_RX_PIN = Pin(PORTC, 7);
#else // defined(DevEBoxH743VI)
  constexpr SpiHandle::Config::Peripheral IMU_SPI_NUM = SpiHandle::Config::Peripheral::SPI_1;
  constexpr Pin IMU_CS_PIN = Pin(PORTA, 4);
  constexpr Pin IMU_SCLK_PIN = Pin(PORTA, 5);
  constexpr Pin IMU_MISO_PIN = Pin(PORTA, 6);
  constexpr Pin IMU_MOSI_PIN = Pin(PORTA, 7);
  constexpr Pin IMU_INT1_PIN = Pin(PORTA, 0);
  constexpr UartHandler::Config::Peripheral DBG_UART_NUM = UartHandler::Config::Peripheral::USART_1;
  constexpr Pin DBG_TX_PIN = Pin(PORTA, 9);
  constexpr Pin DBG_RX_PIN = Pin(PORTA, 10);
  constexpr UartHandler::Config::Peripheral SER_RX_UART_NUM = UartHandler::Config::Peripheral::USART_6;
  constexpr Pin SER_RX_TX_PIN = Pin(PORTC, 6);
  constexpr Pin SER_RX_RX_PIN = Pin(PORTC, 7);  
#endif /* ARDUINO_FC_MatekH743 */

/** Global Hardware access */
UVOSboard         hw;

uint8_t sumbuff[1024];

void UsbCallback(uint8_t* buf, uint32_t* len)
{
    for(size_t i = 0; i < *len; i++)
    {
        sumbuff[i] = buf[i];
    }
}

int main(void)
{
    /** Initialize our hardware */
    hw.Init();

    System::Delay(50);

    hw.usb_handle.Init(UsbHandle::FS_INTERNAL);

    int  tick_cnt = 0;
    bool ledstate = false;
    char buff[512];
    sprintf(buff, "Received:\t%d\r\n", tick_cnt);

    hw.usb_handle.TransmitInternal((uint8_t*)buff, strlen(buff));

    System::Delay(500);

    hw.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_INTERNAL);

    // Create SerialReceiver of type IBUS, provide a parse = TRUE callback
    SerialReceiver ibus_rx(SerialReceiver::IBUS);

    // Config SerialReceiver object UART and initialize it
    SerialReceiver::Config ser_rx_config;
    ser_rx_config.periph = SER_RX_UART_NUM;
    ser_rx_config.rx = SER_RX_RX_PIN;
    ser_rx_config.tx = SER_RX_TX_PIN;

    ibus_rx.Init(ser_rx_config);

    // Start the SerialReceiver
    ibus_rx.StartRx();

    ParsedMessage msg;

    /** Infinite Loop */
    while (1)
    {
        /** Process Serial Rx in the background */
        if (ibus_rx.Listener())
        {
            ibus_rx.GetMessage(&msg);
        }
        /** Print out the first 4 channels from the IBUS message
         * using fixed width formatting */
           sprintf(buff,
                "Ch1: %4d\tCh2: %4d\tCh3: %4d\tCh4: %4d\r\n",
                msg.channels[0],
                msg.channels[1],
                msg.channels[2],
                msg.channels[3]);
        hw.usb_handle.TransmitInternal((uint8_t*) buff, strlen(buff));

        // Wait 100ms
        System::Delay(100);
        hw.SetLed(ledstate = !ledstate);
    }
}