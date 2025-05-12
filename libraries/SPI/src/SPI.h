// SPI.h – Arduino‑style wrapper around your existing SPI driver
#pragma once

#include "per/spi.h"   // uvos::SpiHandle
#include "per/gpio.h"  // uvos::GPIO + Pin helper
#include "uvos_core.h"
#include <cstdint>

namespace uvos {

/* -------------------------------------------------------------------------- */
/*  SPISettings — mirrors Arduino’s SPISettings object                        */
/* -------------------------------------------------------------------------- */
class SPISettings {
public:
    SPISettings(uint32_t clock_hz = 1'000'000UL,
                SpiHandle::Config::ClockPolarity cpol = SpiHandle::Config::ClockPolarity::LOW,
                SpiHandle::Config::ClockPhase    cpha = SpiHandle::Config::ClockPhase::ONE_EDGE,
                uint8_t                          data_bits = 8);

    uint32_t                              clock_hz_;
    SpiHandle::Config::ClockPolarity      cpol_;
    SpiHandle::Config::ClockPhase         cpha_;
    uint8_t                               data_bits_;
};

/* -------------------------------------------------------------------------- */
/*  SPIClass — thin, non‑templated, Arduino‑like wrapper                      */
/* -------------------------------------------------------------------------- */
class SPIClass {
public:
    /** User‑supplied configuration (mirrors TwoWire::Config style). */
    struct Config {
        SpiHandle::Config spi_config{};   ///< underlying peripheral + pins
        Pin               cs{};           ///< optional dedicated CS pin (idle HIGH)
    };

    /** Construct an SPIClass wrapper with 
     *  user‑provided peripheral + optional CS definition. */
    explicit SPIClass(const Config& cfg);

    /* -------- Arduino‑like API ---------- */
    void begin();
    void end();

    void beginTransaction(const SPISettings& settings);
    void endTransaction();

    uint8_t transfer(uint8_t data);
    void    transfer(void* tx, void* rx, size_t length);
    void    transfer(void* buf, size_t length);   ///< in‑place, half‑duplex

    /** Direct access in case advanced functionality is required. */
    SpiHandle& handle() { return spi_; }

private:
    SpiHandle  spi_;
    Config     cfg_{};
    GPIO       cs_gpio_{};
    bool       cs_valid_       = false;
    bool       in_transaction_ = false;
};

} // namespace uvos
