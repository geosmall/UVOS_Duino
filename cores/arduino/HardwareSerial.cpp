#include "HardwareSerial.h"

using namespace uvos_arduino;

/* ----------------------------------------------------------------- */
/*  Constructor: copies user‑supplied config                         */
/* ----------------------------------------------------------------- */
// change signature to take Config not UartHandler::Config
HardwareSerial::HardwareSerial(const Config& cfg)
: cfg_(cfg)
{
}

/* ----------------------------------------------------------------- */
/*  begin() – change baud, init peripheral, start circular RX DMA    */
/* ----------------------------------------------------------------- */
void HardwareSerial::begin(unsigned long baud)
{
    // update baud
    cfg_.uart_config.baudrate = baud;

    // init UART with full config
    uart_.Init(cfg_.uart_config);

    // start DMA‐listen on user buffer
    uart_.DmaListenStart(
        cfg_.dma_buf,
        cfg_.dma_buf_size,
        &HardwareSerial::RxDmaCallback,
        this
    );
}

/* ----------------------------------------------------------------- */
void HardwareSerial::end()
{
    uart_.DmaListenStop();
}

/* ----------------------------------------------------------------- */
/*  DMA callback – runs in IRQ context                               */
/* ----------------------------------------------------------------- */
void HardwareSerial::RxDmaCallback(uint8_t *data,
                                   size_t   size,
                                   void    *ctx,
                                   uvos::UartHandler::Result res)
{
    if(res != uvos::UartHandler::Result::OK) return;

    auto *self = static_cast<HardwareSerial*>(ctx);
    for(size_t i = 0; i < size; ++i)
        self->pushByte(data[i]);
}

/* push one byte into the software ring (drop if full) */
void HardwareSerial::pushByte(uint8_t byte)
{
    const size_t next = (head_ + 1U) % kRxBufSize;
    if(next == tail_)                        // overflow → drop
        return;
    rx_buf_[head_] = byte;
    head_ = next;
}

/* ----------------------------------------------------------------- */
/*  Stream overrides                                                 */
/* ----------------------------------------------------------------- */
int HardwareSerial::available()
{
    int n = (head_ >= tail_)
              ? static_cast<int>(head_ - tail_)
              : static_cast<int>(kRxBufSize - tail_ + head_);
    return n;
}

int HardwareSerial::availableForWrite()
{
    return 64;  // blocking TX always has “space”
}

int HardwareSerial::peek()
{
    return available() ? rx_buf_[tail_] : -1;
}

int HardwareSerial::read()
{
    if(!available()) return -1;
    uint8_t b = rx_buf_[tail_];
    tail_ = (tail_ + 1U) % kRxBufSize;
    return b;
}

void HardwareSerial::flush()
{
    /* nothing to do – BlockingTransmit is synchronous */
}

size_t HardwareSerial::write(uint8_t byte)
{
    uart_.BlockingTransmit(&byte, 1, 100);
    return 1;
}

size_t HardwareSerial::write(const uint8_t *buf, size_t len)
{
    if(!buf || !len) return 0;
    uart_.BlockingTransmit(const_cast<uint8_t*>(buf), len, 100);
    return len;
}
