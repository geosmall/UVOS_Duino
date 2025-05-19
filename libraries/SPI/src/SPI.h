// SPI.h – Arduino‑compatible wrapper around your existing SPI driver (single‑CS via spi_config.pin_config.nss)
#pragma once

#include <Arduino.h>
#include "per/spi.h"        // uvos::SpiHandle driver
#include "per/gpio.h"       // uvos::GPIO helper
#include "uvos_core.h"
#include <cstdint>

using namespace uvos;

/* -------------------------------------------------------------------------- */
/*  Ensure SPI_MODE0‑3 macros exist even if we replaced the stock SPI lib     */
/* -------------------------------------------------------------------------- */
#ifndef SPI_MODE0
#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03
#endif

/* -------------------------------------------------------------------------- */
/*  SPISettings – (clock, dataMode)                                           */
/* -------------------------------------------------------------------------- */
class SPISettings
{
public:
    explicit SPISettings(uint32_t clock = 4'000'000UL,
                         uint8_t  dataMode = SPI_MODE3)
        : clock_(clock), dataMode_(dataMode)
    {}

    uint32_t clock_;   ///< target SCK frequency in Hz
    uint8_t  dataMode_;///< SPI_MODE0…SPI_MODE3
};

/* -------------------------------------------------------------------------- */
/*  SPIClass – thin, non‑templated, Arduino‑like wrapper                      */
/* -------------------------------------------------------------------------- */
class SPIClass
{
public:
    struct Config {
        SpiHandle::Config spi_config{};   ///< underlying peripheral + pins (pin_config.nss = CS)
    };

    explicit SPIClass(const Config& cfg);

    /* ---------- Arduino API ---------- */
    void begin();
    void end();

    void beginTransaction(const SPISettings& settings);
    void endTransaction();

    uint8_t transfer(uint8_t data);
    void    transfer(void* tx, void* rx, size_t length);
    void    transfer(void* buf, size_t length);   ///< in‑place, half‑duplex

    SpiHandle& getHandle() { return spi_; } ///< for low‑level access

private:
    SpiHandle  spi_;
    Config     cfg_{};
    GPIO       cs_gpio_{};
    bool       in_transaction_ = false;
};
