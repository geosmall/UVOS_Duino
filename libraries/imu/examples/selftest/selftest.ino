#include "uvos_brd.h"
#include "imu.h"

extern "C" {
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
}

#include <cstring>

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

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

// Handle we'll use to interact with IMU SPI
SpiHandle spi_handle;

// Structure to configure the IMU SPI instance
SpiHandle::Config spi_conf;

// Create the IMU object
IMU imu{};

// Global print buffer
char buf[128];

static uint8_t spi_freq_mhz = 1;

int main(void)
{
    // Initialize the UVOS board hardware
    hw.Init();

    hw_configure();

    /* Setup message facility to see internal traces from FW */
    INV_MSG_SETUP(MSG_LEVEL, msg_printer);

    INV_MSG(INV_MSG_LEVEL_INFO, "#########################");
    INV_MSG(INV_MSG_LEVEL_INFO, "#   Example Self-Test   #");
    INV_MSG(INV_MSG_LEVEL_INFO, "#########################");

    // Give ICM-42688P some time to stabilize
    System::Delay(5);

    if (imu.Init(spi_handle) != INV_ERROR_SUCCESS) {
        INV_MSG(INV_MSG_LEVEL_INFO, "!!! ERROR : failed to initialize Icm426xx.");
    } else {
        INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx PASS");
    }

    // Perform Self-Test
    int rc = 0;
    int st_result = 0;
    std::array<int, 6> raw_bias = {0};
    rc = imu.RunSelfTest(&st_result, &raw_bias);

    if (rc < 0) {
        INV_MSG(INV_MSG_LEVEL_ERROR, "An error occured while running selftest");
    } else {
        /* Check for GYR success (1 << 0) and ACC success (1 << 1) */
        if (st_result & 0x1) {
            INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest PASS");
        } else {
            INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest FAIL");
        }

        if (st_result & 0x2) {
            INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest PASS");
        } else {
            INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest FAIL");
        }

        INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (dps): x=%f, y=%f, z=%f",
                (float)(raw_bias[0]) / (float)(1 << 16), (float)(raw_bias[1]) / (float)(1 << 16),
                (float)(raw_bias[2]) / (float)(1 << 16));
        INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (g): x=%f, y=%f, z=%f",
                (float)(raw_bias[0 + 3] / (float)(1 << 16)),
                (float)(raw_bias[1 + 3] / (float)(1 << 16)),
                (float)(raw_bias[2 + 3] / (float)(1 << 16)));
    }

    // Infinite loop
    while(1) {
        // Do nothing
    }
}

void hw_configure()
{
    // Configure the Uart Peripheral to print out results
    UartHandler::Config uart_conf;
    uart_conf.periph        = UART_NUM;
    uart_conf.mode          = UartHandler::Config::Mode::TX;
    uart_conf.pin_config.tx = TX_PIN;
    uart_conf.pin_config.rx = RX_PIN;

    // Initialize the uart peripheral and start the DMA transmit
    uart.Init(uart_conf);

    SpiHandle::Config spi_conf;   // Structure to configure the IMU SPI instance

    spi_conf.periph = SpiHandle::Config::Peripheral::SPI_1;
    spi_conf.mode = SpiHandle::Config::Mode::MASTER;
    spi_conf.direction = SpiHandle::Config::Direction::TWO_LINES;
    spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    spi_conf.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;

#ifdef USE_SOFT_NSS
    spi_conf.nss = SpiHandle::Config::NSS::SOFT;
#else
    spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
#endif /* USE_SOFT_NSS */

    spi_conf.pin_config.nss = CS_PIN;
    spi_conf.pin_config.sclk = SCLK_PIN;
    spi_conf.pin_config.miso = MISO_PIN;
    spi_conf.pin_config.mosi = MOSI_PIN;

    // spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_32;
    spi_handle.GetBaudHz(spi_conf.periph, (spi_freq_mhz * 1'000'000), spi_conf.baud_prescaler);

    // Initialize the IMU SPI instance
    spi_handle.Init(spi_conf);
}

/* --------------------------------------------------------------------------------------
 *  Extern functions definition - Invensense to UVOS adapters
 * -------------------------------------------------------------------------------------- */

#ifdef __cplusplus
extern "C" {
#endif

/* Printer function for message facility */
void msg_printer(int level, const char *str, va_list ap)
{
    static char out_str[256]; /* static to limit stack usage */
    unsigned idx = 0;
    const char *s[INV_MSG_LEVEL_MAX] = {
        "", // INV_MSG_LEVEL_OFF
        "[E] ", // INV_MSG_LEVEL_ERROR
        "[W] ", // INV_MSG_LEVEL_WARNING
        "[I] ", // INV_MSG_LEVEL_INFO
        "[V] ", // INV_MSG_LEVEL_VERBOSE
        "[D] ", // INV_MSG_LEVEL_DEBUG
    };
    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
    if (idx >= (sizeof(out_str)))
        return;
    idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
    if (idx >= (sizeof(out_str)))
        return;
    idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
    if (idx >= (sizeof(out_str)))
        return;

    // inv_uart_mngr_puts(LOG_UART_ID, out_str, (unsigned short)idx);
    uart.BlockingTransmit((uint8_t*)out_str, idx);
}

/* Helper function to check RC value and block programm execution */
void check_rc(int rc, const char *msg_context)
{
    if (rc < 0) {
        INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
        while (1)
            ;
    }
}

#ifdef __cplusplus
}
#endif
