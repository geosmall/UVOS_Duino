/* Dps3xxDma.cpp / Dps3xxDma.h
   These files extends Infineon’s arduino-xensiv-dps3xx library (see ../archive)
   to support dsp3xx non-blocking I2C DMA transfers and non-FIFO PRS_RDY polling.
   Interface inspired by INav project’s barometer integration:
   see: https://github.com/iNavFlight/inav
   Class hierarchy: DpsClass -> Dps3xx -> Dps3xxDma
*/

#pragma once

#include "Dps3xx.h"          // base DPS3xx driver
#include "Wire.h"            // uvos::TwoWire wrapper

class Dps3xxDma : public Dps3xx
{
public:
    /**
     * @brief Override base begin() to reset DMA state flag.
     * @param bus          I²C bus instance.
     * @param slaveAddress I²C address of DPS3xx.
     * @return true on successful sensor init, false otherwise.
     */
    bool begin(TwoWire &bus, uint8_t slaveAddress);

    /** 
     * @brief Configure DPS for pressure-only, 32 Hz, OSR ×16, FIFO disabled.
     *        NOTE: Follows INav-style startup sequence.
     * @return DPS__SUCCEEDED or error code on failure.
     */
    int16_t startPressure32HzNoFIFO();

    /**
     * @brief Starts a continuous pressure measurement with specified measurement rate and oversampling rate.
     *
     * @param measureRate      DPS__MEASUREMENT_RATE_1,_2,-->_128
     * @param oversamplingRate DPS__OVERSAMPLING_RATE_1,_2,-->_128
     * @return DPS__SUCCEEDED or error code on failure.
     */
    int16_t startMeasurePressureContNoFIFO(uint8_t measureRate, uint8_t oversamplingRate);

    /* ---------- DMA-enhanced helpers ---------- */

    /**
     * @brief Initiate a non-blocking DMA read of a register block.
     * @param regBlock  Register address + length to fetch.
     * @param dma_buf   User buffer for DMA to fill.
     * @return DPS__SUCCEEDED or error code if busy/invalid.
     */
    int16_t readBlockDMA(RegBlock_t regBlock, uint8_t* dma_buf);

    /**
     * @brief Query whether the previous DMA read has completed.
     * @return true if no DMA in progress or transfer finished.
     */
    bool isReadDone();

    /**
     * @brief Poll the sensor’s PRS_RDY bit for new pressure data.
     * @return true if a fresh pressure sample is available.
     */
    bool pressureSampleIsReady();

    /** 
     * @brief Convert raw 24-bit pressure sample to Pascals.
     * @param raw  Signed 24-bit sample.
     * @return Floating-point pressure in Pa.
     */
    float pressurePaFromRaw(int32_t raw) { return calcPressure(raw); }

private:
    /** 
     * @brief Indicates if an I²C DMA read is currently in progress.
     *        When true, further DMA reads are rejected until complete.
     */
    volatile bool dmaBusy_ { false };
};
