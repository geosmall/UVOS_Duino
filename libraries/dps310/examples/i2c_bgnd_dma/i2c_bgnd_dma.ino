#include "uvos_brd.h"
#include "Dps310.h"

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

// Dps310 Opject
Dps310 Dps310PressureSensor = Dps310();

/* ------------ DMA target buffer ---------------------------------- */
/* 6‑byte block holds three register bytes each for pressure & temp.
 * MUST be in D2 or cache‑cleaned – attribute from your BSP:          */
DMA_BUFFER_MEM_SECTION uint8_t resultBuf[6];

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

    Serial.begin(115200);
    while (!Serial);

    Serial.println("Example i2c_bgnd_dma.ino");

    // initialise sensor; must supply I2C address (0x76 or 0x77)
    if (!Dps310PressureSensor.begin(Wire, 0x76)) {
        Serial.println("Sensor init FAILED!");
        while (1);
    } else {
        Serial.println("Sensor init complete!");
    }

    //temperature measure rate (value from 0 to 7)
    //2^temp_mr temperature measurement results per second
    int16_t temp_mr = 2;
    //temperature oversampling rate (value from 0 to 7)
    //2^temp_osr internal temperature measurements per result
    //A higher value increases precision
    int16_t temp_osr = 2;
    //pressure measure rate (value from 0 to 7)
    //2^prs_mr pressure measurement results per second
    int16_t prs_mr = 2;
    //pressure oversampling rate (value from 0 to 7)
    //2^prs_osr internal pressure measurements per result
    //A higher value increases precision
    int16_t prs_osr = 2;
    //startMeasureBothCont enables background mode
    //temperature and pressure ar measured automatically
    //High precision and hgh measure rates at the same time are not available.
    //Consult Datasheet (or trial and error) for more information
    // int16_t ret = Dps310PressureSensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
    //Use one of the commented lines below instead to measure only temperature or pressure
    //int16_t ret = Dps310PressureSensor.startMeasureTempCont(temp_mr, temp_osr);
    //int16_t ret = Dps310PressureSensor.startMeasurePressureCont(prs_mr, prs_osr);

    /* One‑shot temperature + pressure, oversampling = 1 */
    int16_t ret = Dps310PressureSensor.startMeasureBothCont(DPS__MEASUREMENT_RATE_1,
                  DPS__OVERSAMPLING_RATE_1,
                  DPS__MEASUREMENT_RATE_1,
                  DPS__OVERSAMPLING_RATE_1);

    if (ret != 0) {
        Serial.print("Sensor start FAILED! ret = ");
        Serial.println(ret);
    } else {
        Serial.println("Sensor start successful!");
    }

    // give the sensor time to take its first background sample
    delay(500);

    /* Kick off a DMA read of the RESULT register block (0x00..0x05) */
    RegBlock_t resultReg = { 0x00, 6 };

    ret = Dps310PressureSensor.readBlockDMA(resultReg, resultBuf);
    if (ret != DPS__SUCCEEDED) {
        Serial.println("DMA request failed!");
        while (1) {}
    } else {
        Serial.println("DMA request started!");
    }

    for (;;) { // Loop forever

        /* Poll until the HAL ISR marks DMA finished */
        if (Dps310PressureSensor.isReadDone()) {

            /* ----------------------------------------------------------------
             *  DMA has deposited 6 bytes:  P2 | P1 | P0 | T2 | T1 | T0
             * ---------------------------------------------------------------- */
            int32_t rawPressure = unpack24(&resultBuf[0]);
            int32_t rawTemp     = unpack24(&resultBuf[3]);

            /* Convert with the regular DPS helper – avoids re‑implementing
               the fixed‑point maths here.                                  */
            float pres = Dps310PressureSensor.pressurePaFromRaw(rawPressure);
            float temp = Dps310PressureSensor.temperatureCFromRaw(rawTemp);

            Serial.print("P = ");
            Serial.print(pres);
            Serial.print(" Pa   |   T = ");
            Serial.print(temp);
            Serial.println(" °C");

            delay(2000);

            ret = Dps310PressureSensor.readBlockDMA(resultReg, resultBuf);
            if (ret != DPS__SUCCEEDED) {
                Serial.println("DMA request failed!");
                while (1) {}
            } else {
                Serial.println("DMA request started!");
            }

        }
    }

    return 0;
}
