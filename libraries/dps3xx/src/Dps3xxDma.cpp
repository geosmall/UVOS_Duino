#include "Dps3xxDma.h"
#include "util/dps_config.h"

/* ------------------------------------------------------------------ */
/*      0.  Modify base class begin() to return true if init() OK     */
/* ------------------------------------------------------------------ */
bool Dps3xxDma::begin(TwoWire& bus, uint8_t slaveAddress)
{
    // call base class I²C begin
    DpsClass::begin(bus, slaveAddress);

    // return:
    // true if init() within DpsClass::begin did not set m_initFail
    // else false
    return (m_initFail == 0U);
}

/* ------------------------------------------------------------------ */
/*      1.  INav-style initialisation                                 */
/* ------------------------------------------------------------------ */
int16_t Dps3xxDma::startPressure32HzNoFIFO()
{
    // abort if initialization failed
    if (m_initFail) {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (m_opMode != dps::Mode::IDLE) {
        return DPS__FAIL_TOOBUSY;
    }
    // pressure: 32 Hz MR, OSR 16×
    if (configPressure(DPS__MEASUREMENT_RATE_32, DPS__OVERSAMPLING_RATE_16)) {
        return DPS__FAIL_UNKNOWN;
    }
    // diable result FIFO
    if (disableFIFO()) {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (setOpMode(dps::Mode::CONT_PRS)) {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

int16_t Dps3xxDma::startMeasurePressureContNoFIFO(uint8_t measureRate, uint8_t oversamplingRate)
{
    // abort if initialization failed
    if (m_initFail) {
        return DPS__FAIL_INIT_FAILED;
    }
    // abort if device is not in idling mode
    if (m_opMode != dps::Mode::IDLE) {
        return DPS__FAIL_TOOBUSY;
    }
    // abort if speed and precision are too high
    if (calcBusyTime(measureRate, oversamplingRate) >= DPS3xx__MAX_BUSYTIME) {
        return DPS__FAIL_UNFINISHED;
    }
    // update precision and measuring rate
    if (configPressure(measureRate, oversamplingRate)) {
        return DPS__FAIL_UNKNOWN;
    }
    // diable result FIFO
    if (disableFIFO()) {
        return DPS__FAIL_UNKNOWN;
    }
    // Start measuring in background mode
    if (setOpMode(dps::Mode::CONT_PRS)) {
        return DPS__FAIL_UNKNOWN;
    }
    return DPS__SUCCEEDED;
}

/* ------------------------------------------------------------------ */
/*      2.  Quick helper to poll PRS_RDY                               */
/* ------------------------------------------------------------------ */
bool Dps3xxDma::pressureSampleIsReady()
{
    return readByteBitfield(dps::config_registers[dps::PRS_RDY]) == 1;
}

/* ------------------------------------------------------------------ */
/*      3.  Non-blocking DMA read, and check isReadDone               */
/* ------------------------------------------------------------------ */
int16_t Dps3xxDma::readBlockDMA(RegBlock_t regBlock, uint8_t* dma_buf)
{
    if (!m_i2cbus || !dma_buf || regBlock.length == 0 || dmaBusy_)  return DPS__FAIL_UNKNOWN;

    m_i2cbus->beginTransmission(m_slaveAddress);
    m_i2cbus->write(regBlock.regAddress);
    if (m_i2cbus->endTransmission(false) != 0) {  // repeated-START
        return DPS__FAIL_UNKNOWN;
    }

    // fire off DMA into user buffer dma_buf
    if (!m_i2cbus->requestFromDMA(m_slaveAddress, dma_buf, regBlock.length, true)) {
        return DPS__FAIL_UNKNOWN;
    }

    dmaBusy_ = true; // mark sensor DMA RX as in process
    return DPS__SUCCEEDED;
}

bool Dps3xxDma::isReadDone()
{
    if (!dmaBusy_) return true;          // nothing running

    if (m_i2cbus && m_i2cbus->dmaTransferDone()) {
        dmaBusy_ = false;                // mark sensor DMA RX as finished
        return true;
    }
    return false;                        // still in progress
}
