#include "uvos_brd.h"
#include "icm42688p.h"

#include "inv_main.h"
#include "inv_uart.h"
#include "inv_gpio.h"

#include <cstring>

using namespace uvos;

// Uncomment to use software driven NSS
#define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1000000

#if defined(ARDUINO_FC_MatekH743)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
    constexpr Pin INT1_PIN = Pin(PORTB, 2);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
    constexpr Pin TX_PIN = Pin(PORTA, 9);
    constexpr Pin RX_PIN = Pin(PORTA, 10);
#elif defined(ARDUINO_NUCLEO_H753ZI)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
    constexpr Pin INT1_PIN = Pin(PORTB, 2);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_3;
    constexpr Pin TX_PIN = Pin(PORTD, 8);
    constexpr Pin RX_PIN = Pin(PORTD, 9);
#else // defined(DevEBoxH743VI)
    constexpr Pin CS_PIN = Pin(PORTA, 4);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTA, 7);
    constexpr Pin INT1_PIN = Pin(PORTA, 0);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
    constexpr Pin TX_PIN = Pin(PORTA, 9);
    constexpr Pin RX_PIN = Pin(PORTA, 10);
#endif /* ARDUINO_FC_MatekH743 */

constexpr bool off = 0;
constexpr bool on = 1;

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

    // Create Invn serial spi interface
    inv_icm426xx_serif spi_if = {
        .context = &spi_handle,
    };

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

/* Inv gpio functions */

struct gpio_mapping {
    GPIO_TypeDef* GPIOx;
    GPIO_InitTypeDef GPIO_Init;
    GPIO int_gpio;
    void (*callback) (void *context, unsigned pin_num);
    void* context;
};

struct gpio_mapping gm[INV_GPIO_MAX] = {
    {  // GPIO_INV_SENSOR_INT1
        .GPIOx = nullptr,
        .GPIO_Init = {
            .Pin = 0,
            .Mode = GPIO_MODE_IT_RISING,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_MEDIUM
        },
        .int_gpio = {},
        .callback = 0,
        .context = 0
    },
    {  // GPIO_INV_SENSOR_INT2
        .GPIOx = nullptr,
        .GPIO_Init = {
            .Pin = 0,
            .Mode = GPIO_MODE_IT_RISING,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_MEDIUM
        },
        .int_gpio = {},
        .callback = 0,
        .context = 0
    }
};

// void gpio_common_callback(void* context);

// This callback is passed to GPIO EXTI with SetInterruptCallback() method 
void gpio_common_callback(void* context)
{
    gm[INV_GPIO_INT1].callback(gm[INV_GPIO_INT1].context, INV_GPIO_INT1);
}

// This init function is called from SetupMCUHardware() in inv_main.c,
// meant to set up INT1 pin and connect to interrupt_cb() in inv_main.c
void inv_gpio_sensor_irq_init(unsigned pin_num,
        void (*interrupt_cb)(void * context, unsigned int_num), void * context)
{
    if(pin_num >= INV_GPIO_MAX)
        return;

    gm[pin_num].callback = interrupt_cb;
    gm[pin_num].context = context;

    gm[pin_num].int_gpio.Init(INT1_PIN, GPIO::Mode::INPUT_IT_RISING, GPIO::Pull::NOPULL);
    gm[pin_num].int_gpio.SetInterruptCallback(gpio_common_callback, context);
}

#ifdef __cplusplus
}
#endif