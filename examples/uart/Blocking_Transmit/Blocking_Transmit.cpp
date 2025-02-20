#include "uvos_brd.h"

using namespace uvos;

UVOSboard   hw;
UartHandler uart;

int main(void)
{
    // Initialize the daisy hardware
    hw.Init();

    // Configure the Uart Peripheral
    UartHandler::Config uart_conf;
    uart_conf.periph        = UartHandler::Config::Peripheral::USART_3;
    uart_conf.mode          = UartHandler::Config::Mode::TX;
    uart_conf.pin_config.tx = Pin(PORTD, 8);
    uart_conf.pin_config.rx = Pin(PORTD, 9);

    // Initialize the uart peripheral and start the DMA transmit
    uart.Init(uart_conf);

    uint8_t tx = 0;
    while(1) {
        uart.BlockingTransmit(&tx, 1, 1000);
        tx++;
        System::Delay(50);
    }
}
