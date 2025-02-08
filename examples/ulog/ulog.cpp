#include "uvos_brd.h"
#include "util/ulog.h"
#include <cstdio>

#include "ulog_test.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;
UartHandler uart;

// Assert function called from ulog_test.c
// If any test assert fails, we end up here
extern "C" void __assert_func(const char *file, int line, const char *func, const char *expr)
{
    // If you're using a debugger, trigger a breakpoint.
    __asm__("BKPT #0");

    // Infinite loop to halt the system.
    while (1) {}
}

char buf[128];
int str_len;

void my_console_logger(ulog_level_t severity, char* msg)
{
    uint32_t timestamp = System::GetUs(); // Retrieve the timestamp

    str_len = sprintf(buf, "console: %u [%s]: %s\r\n",
                      timestamp, // Use the timestamp directly
                      ulog_level_name(severity),
                      msg);

    uart.BlockingTransmit((uint8_t*)buf, str_len);
}

void my_file_logger(ulog_level_t severity, char* msg)
{
    uint32_t timestamp = System::GetUs(); // Retrieve the timestamp

    str_len = sprintf(buf, "file: %u [%s]: %s\r\n",
                      timestamp, // Use the timestamp directly
                      ulog_level_name(severity),
                      msg);

    uart.BlockingTransmit((uint8_t*)buf, str_len);
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

    // Initialize the uart peripheral and start the DMA transmit
    uart.Init(uart_conf);

    int arg = 42;

    ULOG_INIT();

    // log messages with a severity of WARNING or higher to the console.  The
    // user must supply a method for my_console_logger, e.g. along the lines
    // of what is shown above.
    ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);

    // log messages with a severity of DEBUG or higher to a file.  The user must
    // provide a method for my_file_logger (not shown here).
    ULOG_SUBSCRIBE(my_file_logger, ULOG_WARNING_LEVEL);

    ULOG_INFO("Info, arg=%d", arg);         // logs to file but not console
    ULOG_CRITICAL("Critical, arg=%d", arg); // logs to file and console

    // dynamically change the threshold for a specific logger
    ULOG_SUBSCRIBE(my_console_logger, ULOG_INFO_LEVEL);

    ULOG_INFO("Info, arg=%d", arg); // logs to file and console

    // remove a logger
    ULOG_UNSUBSCRIBE(my_file_logger);

    ULOG_INFO("Info, arg=%d", arg); // logs to console only

    ulog_test();

    // test passed if we get to here
    str_len = sprintf(buf, "uLog test passed...");
    uart.BlockingTransmit((uint8_t*)buf, str_len);

    // Loop forever
    for (;;) {
        // Set the onboard LED
        hw.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Wait 500ms
        System::Delay(500);
    }
}
