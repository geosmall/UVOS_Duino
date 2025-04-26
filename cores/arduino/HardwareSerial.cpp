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

// push one byte into the software ring (overwrite oldest on full)
void HardwareSerial::pushByte(uint8_t byte)
{
    buffer_.PutWithOverwrite(byte);
}

/* ----------------------------------------------------------------- */
/*  Stream overrides                                                 */
/* ----------------------------------------------------------------- */
int HardwareSerial::available()
{
    return static_cast<int>(buffer_.GetNumElements());
}

int HardwareSerial::availableForWrite()
{
    return 256; // blocking TX always has space
}

int HardwareSerial::peek()
{
    uint8_t b;
    return buffer_.Peek(b) ? static_cast<int>(b) : -1;
}

int HardwareSerial::read()
{
    uint8_t b;
    return buffer_.Get(b) ? static_cast<int>(b) : -1;
}

void HardwareSerial::flush()
{
    /* nothing to do – TX is synchronous */
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
