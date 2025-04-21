/**************************************************************
 * @file HardwareSerial.hpp
 * @brief Arduino‑style, DMA‑backed serial class for STM32H7
 *
 *  – RX: UART in circular‑DMA “listen” mode → lock‑free ring
 *  – TX: non‑blocking DMA with lock‑free ring & chaining
 *
 * 2025‑04‑20
 *************************************************************/
#pragma once

#include <stdint.h>
#include <string.h>

#include "per/uart.h"        // uvos::UartHandler driver
#include "util/ring_buf.h"   // ring_buf::RingBuffer<>
#include "Stream.h"          // Arduino Stream / Print
#include "MF_Serial.h"       // brings in the abstract MF_Serial base

/*****************************************************************
 * SECTION 1  –  DMA / ring‑buffer engine
 *****************************************************************/
namespace mf {

/**
 * @tparam RxRing   RX FIFO depth (bytes)
 * @tparam TxRing   TX FIFO depth (bytes)
 * @tparam DmaChunk Max bytes per single DMA TX transaction
 */
template<size_t RxRing = 1024,
         size_t TxRing = 512,
         size_t DmaChunk = 256>
class SerialUVSEngine : public MF_Serial
{
  public:
    using Uart       = uvos::UartHandler;
    using Result     = typename Uart::Result;
    using Peripheral = typename Uart::Config::Peripheral;

    SerialUVSEngine(Peripheral periph,
                    const uvs_gpio_pin& tx,
                    const uvs_gpio_pin& rx,
                    typename Uart::Config::Mode mode =
                         Uart::Config::Mode::TX_RX)
    {
        cfg_.periph         = periph;
        cfg_.pin_config.tx  = tx;
        cfg_.pin_config.rx  = rx;
        cfg_.mode           = mode;
        /* stopbits, parity, wordlength already defaulted in ctor */
    }

    /* ---------- MF_Serial interface ---------------------------- */
    void begin(int baud) override
    {
        cfg_.baudrate = static_cast<uint32_t>(baud);
        if(uart_.Init(cfg_) != Result::OK)
            return;                               // consider assert/throw

        /* Start circular‑DMA RX */
        uart_.DmaListenStart(dma_rx_, DmaChunk,
                             rx_cb_, this);
    }

    int available() override
    {
        return static_cast<int>(rx_fifo_.GetAvailable());
    }

    int availableForWrite() override
    {
        return static_cast<int>(tx_fifo_.GetFree());
    }

    int read(uint8_t *buf, int len) override
    {
        int cnt = 0;
        while(cnt < len && rx_fifo_.Read(&buf[cnt], 1))
            ++cnt;
        return cnt;
    }

    int write(uint8_t *buf, int len) override
    {
        int pushed = 0;
        /* 1) enqueue into TX ring (producer context)  */
        for(; pushed < len && tx_fifo_.Write(&buf[pushed], 1); ++pushed);

        /* 2) kick DMA if idle */
        __disable_irq();
        if(!tx_busy_ && tx_fifo_.GetAvailable())
            start_next_tx_();
        __enable_irq();

        return pushed;
    }

  protected:
    /* make config accessible to derived Arduino wrapper */
    typename Uart::Config cfg_{};
    bool peek_byte(uint8_t &b) const { return rx_fifo_.Peek(&b, 1); }

  private:
    /* ----------- RX callback (ISR context) -------------------- */
    static void rx_cb_(uint8_t* data,
                       size_t   sz,
                       void*    ctx,
                       Result   res)
    {
        if(res != Result::OK) return;
        auto *self = static_cast<SerialUVSEngine*>(ctx);
        for(size_t i = 0; i < sz; ++i)
            self->rx_fifo_.Write(&data[i], 1);   // drop on overflow
    }

    /* ----------- TX DMA helper & ISR callback ----------------- */
    void start_next_tx_()
    {
        last_tx_len_ = (tx_fifo_.GetAvailable() > DmaChunk)
                       ? DmaChunk
                       : tx_fifo_.GetAvailable();

        if(last_tx_len_ == 0)
            return;

        /* Copy without consuming (Peek) */
        tx_fifo_.Peek(dma_tx_, last_tx_len_);

        tx_busy_ = true;
        uart_.DmaTransmit(dma_tx_, last_tx_len_,
                          nullptr,     /* start‑cb */
                          tx_end_cb_,  /* end‑cb   */
                          this);
    }

    static void tx_end_cb_(void* ctx, Result res)
    {
        auto *self = static_cast<SerialUVSEngine*>(ctx);
        if(res == Result::OK)
            self->tx_fifo_.Skip(self->last_tx_len_);

        self->tx_busy_ = false;

        /* chain next chunk if data pending */
        if(self->tx_fifo_.GetAvailable())
            self->start_next_tx_();
    }

    /* -------------- data members ------------------------------ */
    Uart  uart_;
    /* cfg_ declared earlier (protected) */

    uvos::RingBuf<uint8_t, RxRing> rx_fifo_;
    uvos::RingBuf<uint8_t, TxRing> tx_fifo_;

    alignas(uint32_t) uint8_t dma_rx_[DmaChunk];
    alignas(uint32_t) uint8_t dma_tx_[DmaChunk];

    volatile bool  tx_busy_{false};
    size_t         last_tx_len_{0};
};

} // namespace mf


/*****************************************************************
 * SECTION 2  –  Arduino‑style facade
 *****************************************************************/
#define SERIAL_8N1 0x06
#define SERIAL_8N2 0x0E
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

namespace uvos {

/*----------- Helper: map Arduino SERIAL_* nibble ---------------*/
inline void apply_serial_mode(UartHandler::Config& c, uint16_t mode)
{
    using C = UartHandler::Config;
    switch(mode) {
    default:                /* Fall‑back to 8‑N‑1 */
    case SERIAL_8N1:
        c.wordlength = C::WordLength::BITS_8;
        c.parity     = C::Parity::NONE;
        c.stopbits   = C::StopBits::BITS_1;
        break;
    case SERIAL_8E2:        /* SERIAL_8E2 – SBUS */
        c.wordlength = C::WordLength::BITS_8;
        c.parity     = C::Parity::EVEN;
        c.stopbits   = C::StopBits::BITS_2;
        break;
    }
}

/**
 * @tparam RxRing   RX FIFO depth (bytes)
 * @tparam TxRing   TX FIFO depth (bytes)
 * @tparam DmaChunk Max bytes per single DMA TX transaction
 */
template<size_t RxRing = 1024,
         size_t TxRing = 512,
         size_t DmaChunk = 256>
class HardwareSerial
      : public mf::SerialUVSEngine<RxRing, TxRing, DmaChunk>,
        public Stream
{
    using Base = mf::SerialUVSEngine<RxRing, TxRing, DmaChunk>;
  public:
    using Base::Base;   /* inherit constructors */

    /* Arduino‑style begin(baud, mode) */
    void begin(unsigned long baud,
               uint16_t mode = 0x06 /* SERIAL_8N1 */)
    {
        apply_serial_mode(this->cfg_, mode);
        Base::begin(static_cast<int>(baud));
    }

    void end() {}   /* optional de‑init could go here */

    /* ------------ Stream virtuals ----------------------------- */
    int available()               override { return Base::available(); }
    int availableForWrite()       override { return Base::availableForWrite(); }

    int read()                    override {
        uint8_t b;
        return (Base::read(&b,1)==1) ? b : -1;
    }
    int read(uint8_t *buf, int len) override {
        return Base::read(buf, len);
    }
    int peek()                    override {
        uint8_t byte;
        return this->peek_byte(byte) ? byte : -1;
    }

    void flush()                  override { /* DMA empties TX itself */ }

    /* ------------ Print virtuals ------------------------------ */
    size_t write(uint8_t b)       override { return Base::write(&b,1); }
    size_t write(const uint8_t* buf, size_t sz) override {
        return Base::write(const_cast<uint8_t*>(buf), static_cast<int>(sz));
    }

    using Print::write;   /* keep String/WString overloads visible */
};

} // namespace uvos
