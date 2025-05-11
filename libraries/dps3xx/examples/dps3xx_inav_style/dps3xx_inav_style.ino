/**************************************************************************
 * dps310_inav_style.ino  –  DPS310 demo set up like INAV
 *   * continuous pressure‑only 32 Hz / OSR 16x
 *   * poll PRS_RDY, 3‑byte DMA, MA4 + 2 Hz PT1
 *   * prints baro altitude @ 16 Hz
 **************************************************************************/

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

/* ------------------- objects & state -------------------------------- */
Dps3xxDma baro = Dps3xxDma();

/* moving‑average (box) filter --------------------------*/
static float maBuf[4] = {};      // 4‑point FIFO
static uint8_t maIdx  = 0;
static uint8_t maFill = 0;

/* PT1 state --------------------------------------------*/
static constexpr float PT1_CUTOFF_HZ = 2.0f;
static constexpr float SAMPLE_DT_S   = 1.f / 16.f;          // after decimate‑by‑2
static constexpr float PT1_ALPHA     = 2 * PI * PT1_CUTOFF_HZ * SAMPLE_DT_S
                                       / (1 + 2 * PI* PT1_CUTOFF_HZ* SAMPLE_DT_S);
static float pt1State = 0.f;

/* temp‑refresh timer -----------------------------------*/
uint32_t nextTempRefresh_ms = 1000;        // first refresh 1 s after boot

/* buffer for one DMA read ------------------------------*/
static DMA_BUFFER_MEM_SECTION uint8_t prsRaw[3];

/* helper: pressure (Pa) → altitude (m) -----------------*/
static inline float paToAlt(float pa)
{
    constexpr float SEA_LEVEL_PA = 101325.f;
    return 44330.f * (1.f - powf(pa / SEA_LEVEL_PA, 0.190284f));
}

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

    Wire.begin();          // I2C @ 400 kHz

    if (!baro.begin(Wire, 0x76)) {
        Serial.println("Sensor init FAILED!");
        while (1);
    } else {
        Serial.println("Sensor init complete!");
    }

    /* --- configure: PRS 32 Hz /16x, TEMP 1 Hz /2× just for comp ---- */
    constexpr uint8_t MR_32HZ = 5, MR_1HZ = 0;
    constexpr uint8_t OSR_16  = 4, OSR_2  = 1;

    // first temperature burst (needed for pressure comp)
    delay(50); // Start-up time
    float trash;
    baro.measureTempOnce(trash, OSR_2);

    // continuous pressure‑only
    if (baro.startMeasurePressureContNoFIFO(MR_32HZ, OSR_16) != DPS__SUCCEEDED) {
        Serial.println("startMeasurePressureCont failed");
        while (true) {}
    } else {
        Serial.println("startMeasurePressureCont success");
    }

    Serial.println("DPS310 running (32 Hz, OSR 16x) …");

    // enable PRS‑ready status flag
    baro.setInterruptSources(dps::INT_FLAG_PRS);

    const RegBlock_t prsBlock = dps::registerBlocks[dps::PRS];
    static uint32_t last = 0;

    /* ---------------- MAIN LOOP --------------------------------------- */
    for (;;) {

        /* -------- Poll live PRS_RDY bit ------------------------------- */
        if (baro.pressureSampleIsReady()) {

            /* -------- Kick 3-byte DMA read of PRS registers --------------- */
            if (baro.readBlockDMA(prsBlock, prsRaw) != DPS__SUCCEEDED)
                continue;
            while (!baro.isReadDone()) {}                // spin-wait (replace with event in real FW)

            /* -------- Assemble raw 24-bit value and sign-extend ----------- */
            int32_t rawPressure = unpack24(&prsRaw[0]);

            /* -------- Skip first 4 samples (warm-up) ---------------------- */
            static uint8_t skip = 4;
            if (skip > 0) {
                --skip;
                continue;
            }

            /* -------- Convert to Pa and Altitude -------------------------- */
            float pa  = baro.pressurePaFromRaw(rawPressure);
            float alt = paToAlt(pa);

            /* -------- Prime filters on very first valid sample ------------ */
            static bool primed = false;
            if (!primed) {
                primed = true;
                for (int i = 0; i < 4; ++i) maBuf[i] = alt;   // MA4 buffer
                maFill = 4;  maIdx = 0;
                pt1State = alt;                              // PT1 state
                Serial.print("Startup Alt(m): ");
                Serial.println(alt, 2);
                continue;                                    // next loop
            }

            /* -------- Decimate by 2 (keep every second sample) ------------ */
            static bool keep = false;
            keep = !keep;
            if (!keep) continue;

            /* -------- 4-sample moving average ----------------------------- */
            maBuf[maIdx] = alt;
            maIdx = (maIdx + 1) & 3;
            float sum = maBuf[0] + maBuf[1] + maBuf[2] + maBuf[3];
            float altMA = sum * 0.25f;                       // /4

            /* -------- 2 Hz PT1 low-pass ---------------------------------- */
            pt1State += PT1_ALPHA * (altMA - pt1State);

            /* -------- Print at ~16 Hz ------------------------------------ */
            if (millis() - last >= 62) {
                last = millis();
                Serial.print("Alt(m): ");
                Serial.println(pt1State, 2);
            }
        }

        /* periodic temperature refresh for compensation ---------------- */
        if ((int32_t)(millis() - nextTempRefresh_ms) >= 0) {
            float trash;
            baro.measureTempOnce(trash, 1);      // OSR 2×
            nextTempRefresh_ms += 1000;
        }
    }

    return 0;
}