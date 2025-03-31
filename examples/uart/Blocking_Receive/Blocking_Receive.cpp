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
    // start the Daisy Patch
    hw.Init();

    // set up our UART peripheral
    UartHandler::Config uart_conf;
    uart_conf.periph        = UartHandler::Config::Peripheral::USART_3;
    uart_conf.mode          = UartHandler::Config::Mode::TX_RX;
    uart_conf.pin_config.tx = Pin(PORTD, 8);
    uart_conf.pin_config.rx = Pin(PORTD, 9);

    // initialize the UART peripheral, and start reading
    uart.Init(uart_conf);

    print_msg("Initializing...\r\n\r\n");

    char prn_buf[512];

    while(1)
    {
        uint8_t rx[10] = {0};

        // Blocking rx
        print_msg("Rx start...");
        uart.BlockingReceive(rx, 4, 10000);
        print_msg("Rx end...");

        // Print a title
        print_msg("Uart Blocking Rx");

        // Print the receive buffer contents
        sprintf(prn_buf, "%d %d %d %d\r\n", rx[0], rx[1], rx[2], rx[3]);
        uart.BlockingTransmit((uint8_t*)prn_buf, strlen(prn_buf));

        // Wait 100 ms
        System::Delay(100);
    }
}
