#include "uvos_brd.h"
#include "Dps3xxDma.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
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

static TwoWire::Config defaultWireCfg = [] {
    TwoWire::Config cfg{};
    // select I2C2 and default pins; set default speed to 400 kHz, Master mode
    cfg.i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_2;
    cfg.i2c_config.pin_config.scl = Pin(PORTB, 10);
    cfg.i2c_config.pin_config.sda = Pin(PORTB, 11);
    cfg.i2c_config.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
    cfg.i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    return cfg;
}();
TwoWire Wire(defaultWireCfg);

/**
 * @details This example shows how to read several results for the temperature
 *          and the pressure from the DPS3xx buffer. The DPS3xx can hold up
 *          to 32 value inside the buffer. If this value is reach no further
 *          values are stored.
 */


// Dps3xxDma Object
Dps3xxDma baro = Dps3xxDma();
static uint8_t DMA_BUFFER_MEM_SECTION prsRaw[3] = {0};

/* Helper: convert the 24‑bit big‑endian two’s‑complement      */
static int32_t unpack24(const uint8_t* p)
{
    int32_t v = (int32_t(p[0]) << 16) | (int32_t(p[1]) << 8) | p[2];
    if (v & 0x00800000) v -= 0x01000000;   // sign‑extend 24‑bit value
    return v;
}

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    setup();

    for (;;) {
        loop();
    }

    return 0;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    /*
     * Call begin to initialize baro
     * The parameter 0x76 is the bus address.
     */
    if (!baro.begin(Wire, 0x76)) {
        Serial.println("Sensor init FAILED!");
        while (1);
    } else {
        Serial.println("Sensor init complete!");
    }

    int16_t ret = baro.startPressure32HzNoFIFO();

    if (ret != 0) {
        Serial.print("Init FAILED! ret = ");
        Serial.println(ret);
    } else {
        Serial.println("Init complete!");
    }
}

/* For DMA read of the register block (0x00..0x02) */
RegBlock_t resultRegPRS = { 0x00, 3 };

void loop()
{
    if (baro.pressureSampleIsReady()) {
        if (baro.readBlockDMA(resultRegPRS, prsRaw) == DPS__SUCCEEDED) {
            /* immediately return – sensor continues sampling while DMA runs   */
        }
    }

    if (baro.isReadDone()) {
        int32_t rawPressure = unpack24(&prsRaw[0]);

        /* Convert with the regular DPS helper – avoids re‑implementing
           the fixed‑point maths here.                                  */
        float pres = baro.pressurePaFromRaw(rawPressure);

        Serial.print("P = ");
        Serial.print(pres);
        Serial.println(" Pa");

        delay(250);
    }
}
