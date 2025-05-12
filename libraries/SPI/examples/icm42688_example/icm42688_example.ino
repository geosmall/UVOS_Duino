#include "uvos_brd.h"
#include "SPI.h"

using namespace uvos;

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
    c.uart_config.pin_config.tx = Pin(PORTD, 8);
    c.uart_config.pin_config.rx = Pin(PORTD, 9);
    // DMA buffer
    c.dma_buf      = HardwareSerial_rx_buf;
    c.dma_buf_size = DMA_BUF_SIZE;
    return c;
}();

// Wrapper instance
HardwareSerial Serial(serial_cfg);

/* ======== IMU register map (minimal set) ============================ */
constexpr uint8_t REG_DEVICE_CONFIG   = 0x11;
constexpr uint8_t REG_INT_CONFIG      = 0x14;
constexpr uint8_t REG_PWR_MGMT0       = 0x1F;
constexpr uint8_t REG_GYRO_CONFIG0    = 0x4F;
constexpr uint8_t REG_ACCL_CONFIG0    = 0x50;
constexpr uint8_t REG_WHO_AM_I        = 0x75;   // expected 0x47
constexpr uint8_t REG_ACCEL_DATA_X1   = 0x1F;   // start of 12-byte burst

/* ======== SPI wrapper setup ========================================= */
// Adjust pins / peripheral to match your board layout
static SPIClass::Config spi_cfg = [] {
    SPIClass::Config c{};
    c.spi_config.periph     = SpiHandle::Config::Peripheral::SPI_1;
    c.spi_config.mode       = SpiHandle::Config::Mode::MASTER;
    c.spi_config.direction  = SpiHandle::Config::Direction::TWO_LINES;
    c.spi_config.nss        = SpiHandle::Config::NSS::SOFT;
    c.spi_config.pin_config.sclk = Pin(PORTA, 5);
    c.spi_config.pin_config.miso = Pin(PORTA, 6);
    c.spi_config.pin_config.mosi = Pin(PORTA, 7);
    c.spi_config.pin_config.nss =  Pin(PORTX, 0); // dont-care
    c.cs = Pin(PORTC, 15);     // dedicated SW chip-select
    return c;
}();

SPIClass SPI(spi_cfg);
SPISettings fast(10'000'000U);           // 10 MHz (≤24 MHz allowed)

/* ======== tiny helpers ============================================== */
inline void spiWrite(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { uint8_t(reg & 0x7F), val };
    SPI.transfer(tx, sizeof tx);          // in-place, half-duplex
}

inline uint8_t spiRead(uint8_t reg)
{
    uint8_t tx[2] = { uint8_t(reg | 0x80), 0x00 };
    uint8_t rx[2];
    SPI.transfer(tx, rx, sizeof tx);
    return rx[1];
}

void readSixAxis(int16_t acc[3], int16_t gyr[3])
{
    uint8_t tx[13] = { uint8_t(REG_ACCEL_DATA_X1 | 0x80) };
    uint8_t rx[13] = {0};

    SPI.transfer(tx, rx, sizeof tx);

    for (int i = 0; i < 3; ++i)
        acc[i] = int16_t(rx[1 + i * 2] << 8 | rx[2 + i * 2]);

    for (int i = 0; i < 3; ++i)
        gyr[i] = int16_t(rx[7 + i * 2] << 8 | rx[8 + i * 2]);
}

/* ======== Arduino lifecycle ========================================= */

// Create LED objects
GPIO led0;
Pin led0_pin = Pin(PORTE, 1);

/*
 * \brief Main entry point of Arduino application
 */
int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    // Initialize led0 as an OUTPUT
    led0.Init(led0_pin, GPIO::Mode::OUTPUT);

    setup();

    for (;;) {
        loop();
    }

    return 0;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }                   // wait for USB/serial ready

    SPI.begin();
    SPI.beginTransaction(fast);

    /* --- soft-reset & basic config --- */
    spiWrite(REG_DEVICE_CONFIG, 0x01);
    delay(50);                            // ≥50 ms after reset

    spiWrite(REG_INT_CONFIG,   0x38);     // disable I3C
    spiWrite(REG_PWR_MGMT0,    0x0F);     // low-noise, gyro+accel ON
    spiWrite(REG_GYRO_CONFIG0, 0x06);     // ±2000 dps @1 kHz
    spiWrite(REG_ACCL_CONFIG0, 0x06);     // ±16 g    @1 kHz

    uint8_t who = spiRead(REG_WHO_AM_I);
    if (who != 0x47) {
        Serial.print(F("ICM-42688-P not found! WHO_AM_I=0x"));
        Serial.println(who, HEX);
        while (1) { }                     // halt
    }
    Serial.println(F("ICM-42688-P ready"));
    SPI.endTransaction();
}

void loop()
{
    constexpr float ACC_LSB = 2048.0f;    // 16 g full-scale
    constexpr float GYR_LSB = 16.4f;      // 2000 dps full-scale

    int16_t acc[3], gyr[3];

    SPI.beginTransaction(fast);
    readSixAxis(acc, gyr);
    SPI.endTransaction();

    /* ---- pretty print over Serial ---- */
    Serial.print(F("AX:"));
    Serial.print(acc[0] / ACC_LSB, 2);
    Serial.print(F(" AY:"));
    Serial.print(acc[1] / ACC_LSB, 2);
    Serial.print(F(" AZ:"));
    Serial.print(acc[2] / ACC_LSB, 2);

    Serial.print(F(" | GX:"));
    Serial.print(gyr[0] / GYR_LSB, 1);
    Serial.print(F(" GY:"));
    Serial.print(gyr[1] / GYR_LSB, 1);
    Serial.print(F(" GZ:"));
    Serial.println(gyr[2] / GYR_LSB, 1);

    delay(10);                            // ~100 Hz update
}
