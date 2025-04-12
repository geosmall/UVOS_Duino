#include "uvos_brd.h"
#include <cstring>

using namespace uvos;

UVOSboard   hw;
UartHandler uart;

// Function to print a message over uart
void print_msg(const char* msg)
{
    char buf[128];
    sprintf(buf, "%s\r\n", msg);
    uart.BlockingTransmit((uint8_t*)buf, strlen(buf));
}

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

    print_msg("Blocking transmit test...\r\n\r\n");

    uint8_t tx = 0;
    while(1) {
        uart.BlockingTransmit(&tx, 1, 1000);
        tx++;
        System::Delay(50);
    }
}
