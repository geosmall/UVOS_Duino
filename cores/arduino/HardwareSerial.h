/********************************************************************
 * HardwareSerial.h
 *
 * Arduino‑style serial wrapper for uvos::UartHandler (STM32H743).
 *  – Constructor takes a *fully‑initialised* UartHandler::Config.
 *  – begin(baud) changes only the baud‑rate, then enables the port.
 *  – RX is handled with a circular DMA “listen” + software ring.
 *  – TX is a simple blocking transmit for easy debugging.
 *
 * © 2025 – MIT licence
 ********************************************************************/
#pragma once

#include <cstdint>
#include <cstddef>
#include "Stream.h"           // from STM32duino core you ported
#include "per/uart.h"             // <-- uvos::UartHandler declaration

namespace uvos_arduino
{

class HardwareSerial : public Stream
{
public:
    /*  Constructor – cfg must be fully populated before call  */
    explicit HardwareSerial(const uvos::UartHandler::Config& cfg);

    /*  Non‑copyable (DMA callback points at 'this')            */
    HardwareSerial(const HardwareSerial&)            = delete;
    HardwareSerial& operator=(const HardwareSerial&) = delete;

    /*  Arduino‑style API                                      */
    void    begin(unsigned long baud);
    void    end();

    /*  Stream overrides                                       */
    int     available()               override;
    int     availableForWrite()       override;     // always “space” (blocking TX)
    int     peek()                    override;
    int     read()                    override;
    void    flush()                   override;     // nothing: TX is synchronous
    size_t  write(uint8_t byte)       override;
    size_t  write(const uint8_t *buf, size_t size) override;

    /*  Expose readBytes for MF_SerialPtrWrapper               */
    using Stream::readBytes;

private:
    /* DMA half/TC/IDLE callback */
    static void RxDmaCallback(uint8_t *data,
                              size_t   size,
                              void    *ctx,
                              uvos::UartHandler::Result res);
    void pushByte(uint8_t byte);

    /* constants */
    static constexpr size_t kRxBufSize  = 512;   // software ring buffer
    static constexpr size_t kDmaBufSize = 256;   // DMA circular buffer
    static constexpr int    kTxReserve  = 64;    // pretend space for write()

    /* members */
    uvos::UartHandler           uart_;
    uvos::UartHandler::Config   cfg_;            // copy held privately

    /* RX ring‑buffer management */
    alignas(4) uint8_t  dma_rx_buf_[kDmaBufSize];
    volatile size_t     head_ = 0;
    volatile size_t     tail_ = 0;
    uint8_t             rx_buf_[kRxBufSize];
};

} // namespace uvos_arduino
