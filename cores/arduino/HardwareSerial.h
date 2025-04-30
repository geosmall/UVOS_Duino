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
#include "Stream.h"
#include "per/uart.h"
#include "util/FIFO.h"

namespace uvos
{

class HardwareSerial : public Stream
{
public:
    // Nested Config: hold UART settings + DMA buffer pointer/size
    struct Config
    {
        uvos::UartHandler::Config uart_config;
        uint8_t*                  dma_buf;
        size_t                    dma_buf_size;

        Config() = default;
    };

    /*  Constructor – cfg must be fully populated before call  */
    explicit HardwareSerial(const Config& cfg);

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

    explicit operator bool() const
    {
        return true;
    }

    /*  Expose readBytes from Stream */
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
    static constexpr size_t kTxBufSize  = 256;   // pretend Tx buffer (blocking)

    /* members */
    uvos::UartHandler                             uart_;
    Config                                        cfg_;            // hold user‐supplied config
    uvos::FIFO<uint8_t, kRxBufSize>               buffer_;  // FIFO ring buffer
};

} // namespace uvos
