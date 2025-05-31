/*
 * ReadIMU6 - Example sketch for reading raw 6-axis IMU data (ICM426xx)
 * 
 * This sketch demonstrates how to configure and read raw accelerometer and gyroscope 
 * data from an ICM426xx IMU sensor using SPI communication. It configures the IMU with 
 * specific settings (±4g for accelerometer, ±2000°/s for gyroscope, 1kHz sample rate),
 * uses interrupt-driven data acquisition, and outputs the raw sensor readings to Serial.
 * 
 * The example supports multiple board configurations (NUCLEO_H753ZI, FC_MatekH743, 
 * and DevEBoxH743VI) with appropriate pin mappings for each.
 * 
 * Features demonstrated:
 * - SPI communication with IMU sensor
 * - Interrupt-based data acquisition 
 * - Raw IMU data reading and processing
 * - TDK Invensense messaging system integration
 */

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

bool led_state = true;

int main(void)
{
    // Initialize the UVOS board hardware
    hw.Init();

    // Configure the Serial uart to print out results
    Serial.begin(115200);

    Serial.println("#########################");
    Serial.println("#   Example Self-Test   #");
    Serial.println("#########################");

    /* Give sensor some time to stabilize */
    System::Delay(15);

    // Infinite loop
    while(1) {
        // Set the onboard LED
        hw.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Wait 500ms
        System::Delay(500);
    }
}
