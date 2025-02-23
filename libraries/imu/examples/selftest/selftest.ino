#include "uvos_brd.h"
#include "imu.h"

#include "inv_main.h"
#include "inv_uart.h"

#include <cstring>

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Uncomment to use software driven NSS
#define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1000000

#if defined(ARDUINO_FC_MatekH743)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
    constexpr Pin TX_PIN = Pin(PORTA, 9);
    constexpr Pin RX_PIN = Pin(PORTA, 10);
#elif defined(ARDUINO_NUCLEO_H753ZI)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_3;
    constexpr Pin TX_PIN = Pin(PORTD, 8);
    constexpr Pin RX_PIN = Pin(PORTD, 9);
#else // defined(DevEBoxH743VI)
    constexpr Pin CS_PIN = Pin(PORTA, 4);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTA, 7);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
    constexpr Pin TX_PIN = Pin(PORTA, 9);
    constexpr Pin RX_PIN = Pin(PORTA, 10);
#endif /* ARDUINO_FC_MatekH743 */

constexpr bool off = 0;
constexpr bool on = 1;

// Declare a UVOSboard object called hw
UVOSboard hw;
UartHandler uart;

SpiHandle spi_handle;         // Handle we'll use to interact with IMU SPI
SpiHandle::Config spi_conf;   // Structure to configure the IMU SPI instance

// Global print buffer
char buf[128];

int main(void)
{
    // Initialize the UVOS board hardware
    hw.Init();

    int str_len = sprintf(buf, "Initializing Example SelfTest...\n");
    uart.BlockingTransmit((uint8_t*)buf, str_len);

    // Give ICM-42688P some time to stabilize
    System::Delay(5);

    // Call the main function of the Invensense example
    inv_main();

    // Infinite loop
    while(1) {
        // Do nothing
    }
}

/* --------------------------------------------------------------------------------------
 *  Extern functions definition - Invensense to UVOS adapters
 * -------------------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

/* This variable contains the number of nested calls to disable_irq */
static uint32_t sDisableIntCount = 0;

void inv_disable_irq(void)
{
    if(sDisableIntCount == 0) {
        __disable_irq();
    }
    sDisableIntCount ++;
}

void inv_enable_irq(void)
{
    sDisableIntCount --;
    if(sDisableIntCount == 0) {
        __enable_irq();
    }
}

int inv_uart_mngr_puts(inv_uart_num_t uart_num, const char* s, unsigned short l)
{
    if (uart.BlockingTransmit((uint8_t*)s, l) == UartHandler::Result::ERR) {
        return 0;
    } else {
        return 1;
    }
}

uint64_t inv_timer_get_counter(unsigned timer_num)
{
    return System::GetUs();
}

void inv_delay_us(uint32_t us)
{
    System::DelayUs(us);
}

#ifdef __cplusplus
}
#endif
