/*
 * IMU Self-Test - Example sketch for running self-test on ICM426xx IMU sensor
 * 
 * This sketch demonstrates how to initialize an ICM426xx IMU sensor over SPI and
 * run the internal self-test procedure. The self-test checks if the accelerometer
 * and gyroscope sensors are functioning properly. Results from the self-test are
 * displayed, including pass/fail status for both sensors and the calculated bias values.
 * 
 * The example supports multiple board configurations (NUCLEO_H753ZI, FC_MatekH743, 
 * and DevEBoxH743VI) with appropriate pin mappings for each.
 * 
 * Features demonstrated:
 * - SPI communication with IMU sensor
 * - Running the built-in self-test procedure
 * - Interpreting self-test results
 * - TDK Invensense messaging system integration
 */

#include "uvos_brd.h"
#include "imu.h"
#include "SPI.h"

extern "C" {
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/RingBuffer.h"
#include "Invn/EmbUtils/ErrorHelper.h"
}

#include <cstring>

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

// Uncomment to use software driven NSS
#define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1'000'000

#if defined(ARDUINO_NUCLEO_H753ZI) || defined(ARDUINO_FC_MatekH743)
    constexpr Pin CS_PIN = Pin(PORTC, 15);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTD, 7);
    constexpr Pin INT1_PIN = Pin(PORTB, 2);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_3;
    constexpr Pin TX_PIN = Pin(PORTD, 8);
    constexpr Pin RX_PIN = Pin(PORTD, 9);
#else // DevEBoxH743VI
    constexpr Pin CS_PIN = Pin(PORTA, 4);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTA, 7);
    constexpr Pin INT1_PIN = Pin(PORTA, 0);
    constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_1;
    constexpr Pin TX_PIN = Pin(PORTA, 9);
    constexpr Pin RX_PIN = Pin(PORTA, 10);
#endif /* ARDUINO_NUCLEO_H753ZI or ARDUINO_FC_MatekH743 */

constexpr bool off = 0;
constexpr bool on = 1;

// Declare a UVOSboard object called hw
UVOSboard hw;

static constexpr size_t DMA_BUF_SIZE = 256;   // DMA rx buffer size
static uint8_t DMA_BUFFER_MEM_SECTION HardwareSerial_rx_buf[DMA_BUF_SIZE] = {0};

// Build full HardwareSerial::Config with nested lambda
static const HardwareSerial::Config serial_cfg = [] {
    HardwareSerial::Config c{};
    // UART settings
    c.uart_config.periph        = UartHandler::Config::Peripheral::USART_3;
    c.uart_config.mode          = UartHandler::Config::Mode::TX_RX;
    c.uart_config.pin_config.tx = TX_PIN;
    c.uart_config.pin_config.rx = RX_PIN;
    // DMA buffer
    c.dma_buf      = HardwareSerial_rx_buf;
    c.dma_buf_size = DMA_BUF_SIZE;
    return c;
}();

// Wrapper instance
HardwareSerial Serial(serial_cfg);

static SPIClass::Config spi_conf = [] {
    SPIClass::Config cfg{};
    cfg.spi_config.periph = SpiHandle::Config::Peripheral::SPI_1;
    cfg.spi_config.mode = SpiHandle::Config::Mode::MASTER;
    cfg.spi_config.direction = SpiHandle::Config::Direction::TWO_LINES;
    cfg.spi_config.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    cfg.spi_config.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;
    cfg.spi_config.nss = SpiHandle::Config::NSS::SOFT;
    cfg.spi_config.nss_pulse = SpiHandle::Config::NSSPulseMode::DISABLE;
    cfg.spi_config.pin_config.nss = CS_PIN;
    cfg.spi_config.pin_config.sclk = SCLK_PIN;
    cfg.spi_config.pin_config.miso = MISO_PIN;
    cfg.spi_config.pin_config.mosi = MOSI_PIN;
    cfg.spi_config.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_64;
    return cfg;
}();
SPIClass SPIbus(spi_conf);   // Handle we'll use to interact with IMU SPI bus

// Create the IMU object
IMU imu{};

// Global print buffer
char buf[128];

int main(void)
{
    // Initialize the UVOS board hardware
    hw.Init();

    // Configure the Serial uart to print out results
    Serial.begin(115200);

    /* Setup TDK messaging facility to see internal traces from FW */
    INV_MSG_SETUP(MSG_LEVEL, msg_printer);

    INV_MSG(INV_MSG_LEVEL_INFO, "#########################");
    INV_MSG(INV_MSG_LEVEL_INFO, "#   Example Self-Test   #");
    INV_MSG(INV_MSG_LEVEL_INFO, "#########################");

    /* Start the SPI bus */
    SPIbus.begin();

    /* Give IMU some time to stabilize */
    System::Delay(15);

    if (imu.Init(SPIbus) != IMU::Result::OK) {
        INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
        while(1);
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

    // uart.BlockingTransmit((uint8_t*)out_str, idx);
    Serial.write((uint8_t*)out_str, idx);
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
