/* -------------------------------------------------------------------------- */
/*  SPI.cpp – implementation                                                  */
/* -------------------------------------------------------------------------- */
#include "SPI.h"
// #include "stm32h7xx_hal.h"   // for prescaler enums if needed

using namespace uvos;

// --------------------------- SPISettings -----------------------------------
SPISettings::SPISettings(uint32_t clock_hz,
                         SpiHandle::Config::ClockPolarity cpol,
                         SpiHandle::Config::ClockPhase    cpha,
                         uint8_t                          data_bits)
    : clock_hz_(clock_hz)
    , cpol_(cpol)
    , cpha_(cpha)
    , data_bits_(data_bits)
{}

// ----------------------------- SPIClass ------------------------------------
SPIClass::SPIClass(const SPIClass::Config& cfg)
    : cfg_(cfg)
    , cs_valid_(cfg.cs.port != UVS_GPIOX)
{
    if (cs_valid_) {
        GPIO::Config gcfg;
        gcfg.pin  = cfg.cs;
        gcfg.mode = GPIO::Mode::OUTPUT;
        gcfg.pull = GPIO::Pull::NOPULL;
        cs_gpio_.Init(gcfg);
        cs_gpio_.Write(true);   // idle‑high (device not selected)
    }
}

void SPIClass::begin()
{
    spi_.Init(cfg_.spi_config);
}

void SPIClass::end()
{
    // Provide if you wrap HAL_SPI_DeInit().  For now no‑op.
}

void SPIClass::beginTransaction(const SPISettings& s)
{
    if (in_transaction_) return;

    if (cs_valid_)
        cs_gpio_.Write(false);  // assert CS

    // Clone stored config and adjust to requested settings.
    SpiHandle::Config dyn = cfg_.spi_config;
    dyn.clock_polarity = s.cpol_;
    dyn.clock_phase    = s.cpha_;
    dyn.datasize       = s.data_bits_;

    // Helper: translate desired Hz to the nearest prescaler.
    SpiHandle::Config::BaudPrescaler presc;
    spi_.GetBaudHz(dyn.periph, s.clock_hz_, presc);
    dyn.baud_prescaler = presc;

    spi_.Init(dyn);
    in_transaction_ = true;
}

void SPIClass::endTransaction()
{
    if (!in_transaction_) return;

    if (cs_valid_)
        cs_gpio_.Write(true);   // de‑assert CS

    in_transaction_ = false;
}

uint8_t SPIClass::transfer(uint8_t data)
{
    uint8_t rx = 0;
    spi_.BlockingTransmitAndReceive(&data, &rx, 1, 100);
    return rx;
}

void SPIClass::transfer(void* tx, void* rx, size_t len)
{
    if (len == 0) return;
    spi_.BlockingTransmitAndReceive(static_cast<uint8_t*>(tx),
                                    static_cast<uint8_t*>(rx),
                                    len,
                                    100);
}

void SPIClass::transfer(void* buf, size_t len)
{
    if (len == 0) return;
    spi_.BlockingTransmitAndReceive(static_cast<uint8_t*>(buf),
                                    static_cast<uint8_t*>(buf),
                                    len,
                                    100);
}
