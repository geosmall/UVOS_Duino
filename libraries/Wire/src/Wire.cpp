/**
 * @file Wire.cpp
 * @brief Implementation of TwoWire using uvos::FIFO for buffering.
 */
#include "Wire.h"
#include <cstring>

namespace uvos
{

/* ------------------------------------------------------------------ */
/*          private helpers                                           */
/* ------------------------------------------------------------------ */
uint32_t TwoWire::toHalSpeed(uint32_t hz)
{
    if (hz >= 800'000) return 1'000'000;
            if(hz >= 350'000) return 400'000;
                                         return 100'000;
}

/* ------------------------------------------------------------------ */
/*          public API                                                */
/* ------------------------------------------------------------------ */
// Constructor: copies user‑supplied config
TwoWire::TwoWire(const TwoWire::Config& cfg)
    : cfg_(cfg)
{
}

// Master mode: init I2C peripheral
bool TwoWire::begin(uint32_t clockHz)
{
    // update stored config
    cfg_.i2c_config.speed = (toHalSpeed(clockHz) == 1'000'000)
                            ? I2CHandle::Config::Speed::I2C_1MHZ
                            : (toHalSpeed(clockHz) == 400'000)
                               ? I2CHandle::Config::Speed::I2C_400KHZ
                               : I2CHandle::Config::Speed::I2C_100KHZ;
                               cfg_.i2c_config.mode  = I2CHandle::Config::Mode::I2C_MASTER;

                               return (i2c_.Init(cfg_.i2c_config) == I2CHandle::Result::OK);
                           }

                               // Slave mode: init I2C peripheral
                               // Note: selfAddr is 7-bit address (0x00-0x7F)
                               bool TwoWire::begin(uint8_t selfAddr, uint32_t clockHz)
                               {
                               // update stored config
                               cfg_.i2c_config.speed   = (toHalSpeed(clockHz) == 1'000'000)
                               ? I2CHandle::Config::Speed::I2C_1MHZ
                               : (toHalSpeed(clockHz) == 400'000)
                            ? I2CHandle::Config::Speed::I2C_400KHZ
                            : I2CHandle::Config::Speed::I2C_100KHZ;
    cfg_.i2c_config.mode    = I2CHandle::Config::Mode::I2C_SLAVE;
    cfg_.i2c_config.address = selfAddr & 0x7F;

    return (i2c_.Init(cfg_.i2c_config) == I2CHandle::Result::OK);
}

void TwoWire::end() { /* No explicit de-init path needed */ }

void TwoWire::setClock(uint32_t hz)
{
    // if in master mode, just re‐init as master; otherwise re‑init as slave with the same address
    if (cfg_.i2c_config.mode == I2CHandle::Config::Mode::I2C_MASTER) {
        begin(hz);
    } else {
        begin(cfg_.i2c_config.address, hz);
    }
}

/* ---------------- Master TX ---------------- */
void TwoWire::beginTransmission(uint8_t address)
{
    txAddress_ = address & 0x7F;
    txFifo_.Clear();
}

size_t TwoWire::write(uint8_t byte)
{
    return txFifo_.PutIfNotFull(byte) ? 1 : 0;
}

size_t TwoWire::write(const uint8_t* buf, size_t len)
{
    size_t written = 0;
    while (written < len && txFifo_.PutIfNotFull(buf[written]))
        ++written;
    return written;
}

uint8_t TwoWire::endTransmission(bool /*stopBit*/)
{
    // Pull bytes out of TX FIFO into a stack buffer (bounded by capacity)
    uint8_t txBuf[kTxCapacity];
    size_t  i = 0;
    uint8_t b;
    while (i < kTxCapacity && txFifo_.Get(b))
        txBuf[i++] = b;

    auto res = i2c_.TransmitBlocking(txAddress_, txBuf, static_cast<uint16_t>(i), 100);
    return (res == I2CHandle::Result::OK) ? 0 : 4;  // Arduino: 0 = OK, 4 = “other error”
}

/* ---------------- Master RX ---------------- */
uint32_t TwoWire::requestFrom(uint8_t address, uint32_t len, bool /*stopBit*/)
{
    if (len > kRxCapacity) len = kRxCapacity;

    // flush old data
    rxFifo_.Clear();

    uint8_t tmp[kRxCapacity];
    auto res = i2c_.ReceiveBlocking(address & 0x7F, tmp, static_cast<uint16_t>(len), 100);
    if (res != I2CHandle::Result::OK)
        return 0;

    // push into FIFO
    for (uint32_t i = 0; i < len; ++i)
        rxFifo_.PutWithOverwrite(tmp[i]);

    return len;
}

/* ------------DMA Master RX ----------------- */
void TwoWire::DmaCallback(void* ctx, I2CHandle::Result res)
{
    auto self = static_cast<TwoWire*>(ctx);

    self->dma_.busy = false;                 // mark DMA finished

    if (res == I2CHandle::Result::OK) {
        for (uint16_t i = 0; i < self->dma_.len; ++i)
            self->rxFifo_.PutWithOverwrite(self->dma_.buf[i]);
    }
}

bool TwoWire::requestFromDMA(uint8_t  address,
                             uint8_t* buf,
                             uint32_t len,
                             bool     /*stopBit*/)
{
    if (dma_.busy || !buf || len == 0 || len > 0xFFFFu)
        return false;

    dma_.busy = true;
    dma_.buf  = buf;
    dma_.len  = static_cast<uint16_t>(len);

    auto res = i2c_.ReceiveDma(address & 0x7F,
                               buf,
                               dma_.len,
                               &TwoWire::DmaCallback,   // <- one function
                               this);                   //    context pointer

    if (res != I2CHandle::Result::OK) {
        dma_.busy = false;
        return false;
    }
    return true;
}

bool TwoWire::dmaTransferDone() const
{
    return !dma_.busy;
}

/* ------------- Stream primitives ------------ */
int TwoWire::available() { return static_cast<int>(rxFifo_.GetNumElements()); }

int TwoWire::peek()
{
    uint8_t b;
    return rxFifo_.Peek(b) ? b : -1;
}

int TwoWire::read()
{
    uint8_t b;
    return rxFifo_.Get(b) ? b : -1;
}

/* ---------------- Global instance ----------- */
//static const TwoWire::Config defaultWireCfg{};
// static TwoWire::Config defaultWireCfg = [] {
//     TwoWire::Config cfg{};
//     // select I2C2 and default pins; set default speed to 400 kHz, Master mode
//     cfg.i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_2;
//     cfg.i2c_config.pin_config.scl = Pin(PORTB, 10);
//     cfg.i2c_config.pin_config.sda = Pin(PORTB, 11);
//     cfg.i2c_config.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
//     cfg.i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
//     return cfg;
// }();
// TwoWire Wire(defaultWireCfg);

} // namespace uvos
