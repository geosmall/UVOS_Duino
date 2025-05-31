#include "uvos_brd.h"
#include "SPI.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Uncomment to use software driven NSS
// #define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1'000'000

#if defined(ARDUINO_NUCLEO_H753ZI) || defined(ARDUINO_FC_MatekH743)
    constexpr Pin CS_PIN = Pin(PORTE, 2);
    constexpr Pin SCLK_PIN = Pin(PORTB, 3);
    constexpr Pin MISO_PIN = Pin(PORTB, 4);
    constexpr Pin MOSI_PIN = Pin(PORTB, 5);
    // constexpr Pin INT1_PIN = Pin(PORTB, 2);
#else // DevEBoxH743VI
    constexpr Pin CS_PIN = Pin(PORTA, 4);
    constexpr Pin SCLK_PIN = Pin(PORTA, 5);
    constexpr Pin MISO_PIN = Pin(PORTA, 6);
    constexpr Pin MOSI_PIN = Pin(PORTA, 7);
    // constexpr Pin INT1_PIN = Pin(PORTA, 0);
#endif /* ARDUINO_NUCLEO_H753ZI or ARDUINO_FC_MatekH743 */
constexpr UartHandler::Config::Peripheral UART_NUM = UartHandler::Config::Peripheral::USART_3;
constexpr Pin TX_PIN = Pin(PORTD, 8);
constexpr Pin RX_PIN = Pin(PORTD, 9);

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
    c.uart_config.periph        = UART_NUM;
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
    cfg.spi_config.periph = SpiHandle::Config::Peripheral::SPI_3;
    cfg.spi_config.mode = SpiHandle::Config::Mode::MASTER;
    cfg.spi_config.direction = SpiHandle::Config::Direction::TWO_LINES;
    cfg.spi_config.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    cfg.spi_config.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;
#if defined(USE_SOFT_NSS)
    cfg.spi_config.nss = SpiHandle::Config::NSS::SOFT;
#else
    cfg.spi_config.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
#endif
    cfg.spi_config.nss_pulse = SpiHandle::Config::NSSPulseMode::DISABLE;
    cfg.spi_config.pin_config.nss = CS_PIN;
    cfg.spi_config.pin_config.sclk = SCLK_PIN;
    cfg.spi_config.pin_config.miso = MISO_PIN;
    cfg.spi_config.pin_config.mosi = MOSI_PIN;
    cfg.spi_config.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_64;
    return cfg;
}();
SPIClass SPIbus(spi_conf);   // Handle we'll use to interact with IMU SPI bus

/**
 * @brief Chip select pin (using software driven CS).
 */
GPIO csPin_;

bool led_state = true;

int main(void)
{
    // Initialize the UVOS board hardware
    hw.Init();

    // Configure the Serial uart to print out results
    Serial.begin(115200);

    Serial.println("#########################");
    Serial.println("#   w25qxx self-test    #");
    Serial.println("#########################");

}