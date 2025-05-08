/**
 * @file Wire.h
 * @brief Arduino-style I²C wrapper that uses uvos::FIFO for buffering.
 */
#pragma once

#include "Stream.h"          // Arduino core
#include "per/i2c.h"         // uvos::I2CHandle
#include "util/FIFO.h"       // uvos::FIFO
#include <cstdint>

namespace uvos
{

class TwoWire : public Stream
{
public:
    static constexpr size_t kTxCapacity = 64;   ///< bytes
    static constexpr size_t kRxCapacity = 64;   ///< bytes

    // new config struct
    struct Config {
        I2CHandle::Config i2c_config;
        uint8_t*          tx_buf        = nullptr;
        size_t            tx_buf_size   = kTxCapacity;
        uint8_t*          rx_buf        = nullptr;
        size_t            rx_buf_size   = kRxCapacity;
    };

    // ctor takes user‑supplied Config
    explicit TwoWire(const Config& cfg);

    /* ------ Arduino public API ------ */
    bool   begin(uint32_t clockHz = 400'000);
    bool   begin(uint8_t selfAddr, uint32_t clockHz);
    void   end();
    void   setClock(uint32_t hz);

    void   beginTransmission(uint8_t address);
    uint8_t endTransmission(bool stopBit = true);

    uint32_t requestFrom(uint8_t address, uint32_t len, bool stopBit = true);

    /**  Start an asynchronous read into a caller‑supplied buffer.
     *   @param address  7‑bit I²C slave address.
     *   @param buf      Pointer to user buffer.  **Must** reside in D2 or the
     *                   caller must call uvs_dma_clear_cache_for_buffer().
     *   @param len      Number of bytes to read.
     *   @param stopBit  Generate STOP after the read (typical Arduino default).
     *   @return true    DMA successfully queued; false otherwise.
     */
    bool requestFromDMA(uint8_t address,
                        uint8_t* buf,
                        uint32_t len,
                        bool     stopBit = true);

    /**  Returns true when the last requestFromDMA() has fully finished
     *   and the data are now in *buf.  */
    bool dmaTransferDone() const;

    int     available() override;
    int     read() override;
    int     peek() override;
    void    flush() override {}
    size_t  write(uint8_t byte) override;
    size_t  write(const uint8_t* buf, size_t len) override;

private:
    I2CHandle                i2c_;
    Config                   cfg_;
    uint8_t                  txAddress_{0};

    FIFO<uint8_t, kTxCapacity> txFifo_;
    FIFO<uint8_t, kRxCapacity> rxFifo_;

    struct DmaState {
        volatile bool busy  = false;      ///< true while a DMA is running
        uint8_t*      buf   = nullptr;    ///< user‑supplied target buffer
        uint16_t      len   = 0;          ///< bytes requested
    } dma_;

    /* helpers */
    uint32_t toHalSpeed(uint32_t hz);
    static void DmaCallback(void* ctx, I2CHandle::Result res);
};

/* Global “Wire” instance **********************************************/
// extern TwoWire Wire;

} // namespace uvos
