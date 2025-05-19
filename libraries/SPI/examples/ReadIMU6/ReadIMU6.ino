#include "uvos_brd.h"
#include "imu2.h"
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

/*
 * ICM mounting matrix
 * Coefficients are coded as Q30 integer
 */
static int32_t icm_mounting_matrix[9] = { (1 << 30), 0, 0, 0, (1 << 30), 0, 0, 0, (1 << 30) };

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

/* Flag set from icm426xx device irq handler */
static volatile bool irq_from_device = false;

// Declare a UVOSboard object called hw
UVOSboard hw;
UartHandler uart;

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

// INT1 Interrupt pin
GPIO intGpio;

// Global print buffer
char buf[128];

/* Buffer to keep track of the timestamp when icm426xx data ready interrupt fires. */
RINGBUFFER(timestamp_buffer, 64, uint64_t);

/*
 * Icm426xx interrupt handler.
 * Function is executed when an Icm426xx interrupt rises on MCU.
 * This function get a timestamp and store it in the timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * are implemented for shared variable timestamp_buffer.
 */
void ext_interrupt_cb(void *context)
{
    (void)context;

    // Read timestamp from the System uSec timer
    uint64_t timestamp = System::GetUs();

    if (!RINGBUFFER_FULL(&timestamp_buffer))
    {
        RINGBUFFER_PUSH(&timestamp_buffer, &timestamp);
    }

    irq_from_device = true;
}

int main(void)
{
    int rc = 0;
    uint64_t irq_timestamp = 0;

    // Initialize the UVOS board hardware
    hw.Init();

    // Configure the Uart Peripheral to print out results
    UartHandler::Config uart_conf;
    uart_conf.periph        = UART_NUM;
    uart_conf.mode          = UartHandler::Config::Mode::TX;
    uart_conf.pin_config.tx = TX_PIN;
    uart_conf.pin_config.rx = RX_PIN;

    // Initialize the uart peripheral and start the DMA transmit
    uart.Init(uart_conf);

    /* Setup message facility to see internal traces from FW */
    INV_MSG_SETUP(MSG_LEVEL, msg_printer);

    INV_MSG(INV_MSG_LEVEL_INFO, "##################################");
    INV_MSG(INV_MSG_LEVEL_INFO, "#   Example Raw data registers   #");
    INV_MSG(INV_MSG_LEVEL_INFO, "##################################");

    /* Give ICM-42688P some time to stabilize */
    System::Delay(15);

    /* Start the SPI bus */
    SPIbus.begin();

    if (imu.Init(SPIbus) != IMU::Result::OK) {
        INV_MSG(INV_MSG_LEVEL_INFO, "!!! ERROR : failed to initialize Icm426xx.");
    } else {
        INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx PASS");
    }

    /* Configure IMU object */
    /* /!\ In this example, the data output frequency will be the faster  between Accel and Gyro odr */
    imu.ConfigureInvDevice(IMU::gpm4, IMU::dps2000, IMU::accel_odr1k, IMU::gyr_odr1k);

    RINGBUFFER_CLEAR(&timestamp_buffer);

    intGpio.SetInterruptCallback(ext_interrupt_cb, nullptr);
    intGpio.Init(INT1_PIN, GPIO::Mode::INPUT_IT_RISING, GPIO::Pull::NOPULL);

    std::array<int16_t, 6> buffer;

    do {
        /* Poll device for data */
        if (irq_from_device) {
            // rc = imu.ReadDataFromRegisters();
            rc = imu.ReadIMU6(buffer);

            __disable_irq();
            if (!RINGBUFFER_EMPTY(&timestamp_buffer)) {
                RINGBUFFER_POP(&timestamp_buffer, &irq_timestamp);
            }
            irq_from_device = false;
            __enable_irq();

            INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, %d, %d", (uint32_t)irq_timestamp,
                    buffer[0], buffer[1], buffer[2],
                    buffer[3], buffer[4], buffer[5]);
        }

    } while (1);
}

/* --------------------------------------------------------------------------------------
 *  Static functions definition
 * -------------------------------------------------------------------------------------- */

static void apply_mounting_matrix(const int32_t matrix[9], int16_t raw[3])
{
    unsigned i;
    int64_t  data_q30[3];

    for (i = 0; i < 3; i++) {
        data_q30[i] = ((int64_t)matrix[3 * i + 0] * raw[0]);
        data_q30[i] += ((int64_t)matrix[3 * i + 1] * raw[1]);
        data_q30[i] += ((int64_t)matrix[3 * i + 2] * raw[2]);
    }
    raw[0] = (int16_t)(data_q30[0] >> 30);
    raw[1] = (int16_t)(data_q30[1] >> 30);
    raw[2] = (int16_t)(data_q30[2] >> 30);
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
