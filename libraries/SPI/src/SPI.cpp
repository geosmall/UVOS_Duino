/* -------------------------------------------------------------------------- */
/*  SPI.cpp – implementation                                                  */
/* -------------------------------------------------------------------------- */
#include "SPI.h"
// #include "stm32h7xx_hal.h"

using namespace uvos;

/* ------------------------- helper: mode → CPOL/CPHA ----------------------- */
static inline void decodeMode(uint8_t mode,
                              SpiHandle::Config::ClockPolarity& cpol,
                              SpiHandle::Config::ClockPhase&    cpha)
{
    switch (mode & 0x03) {
    case SPI_MODE0: cpol = SpiHandle::Config::ClockPolarity::LOW;  cpha = SpiHandle::Config::ClockPhase::ONE_EDGE;  break;
    case SPI_MODE1: cpol = SpiHandle::Config::ClockPolarity::LOW;  cpha = SpiHandle::Config::ClockPhase::TWO_EDGE;  break;
    case SPI_MODE2: cpol = SpiHandle::Config::ClockPolarity::HIGH; cpha = SpiHandle::Config::ClockPhase::ONE_EDGE;  break;
    default:        cpol = SpiHandle::Config::ClockPolarity::HIGH; cpha = SpiHandle::Config::ClockPhase::TWO_EDGE;  break; // MODE3
    }
}

/* ----------------------------- SPIClass ----------------------------------- */
SPIClass::SPIClass(const SPIClass::Config& cfg)
    : cfg_(cfg)
{
}

// Uncomment to use software driven NSS
#define USE_SOFT_NSS
#define DESIRED_SPI_FREQ 1'000'000

constexpr Pin SCLK_PIN = Pin(PORTA, 5);
constexpr Pin MISO_PIN = Pin(PORTA, 6);
constexpr Pin MOSI_PIN = Pin(PORTD, 7);
constexpr Pin CS_PIN   = Pin(PORTC, 15);
constexpr Pin INT1_PIN = Pin(PORTB, 2);

void SPIClass::begin()
{
    spi_.Init(cfg_.spi_config);
}

void SPIClass::end()
{
    // Optional HAL_SPI_DeInit()
}

void SPIClass::beginTransaction(const SPISettings& s)
{
    if (in_transaction_) return;

    SpiHandle::Config dyn = cfg_.spi_config;

    decodeMode(s.dataMode_, dyn.clock_polarity, dyn.clock_phase);
    // dyn.datasize = 8;

    SpiHandle::Config::BaudPrescaler presc;
    spi_.GetBaudHz(dyn.periph, s.clock_, presc);
    dyn.baud_prescaler = presc;

    // spi_.Init(dyn);
    in_transaction_ = true;
}

void SPIClass::endTransaction()
{
    if (!in_transaction_) return;
    in_transaction_ = false;
}

uint8_t SPIClass::transfer(uint8_t data)
{
    uint8_t rx_byte = 0;
    spi_.BlockingTransmitAndReceive(&data, &rx_byte, 1, 100);
    return rx_byte;
}

void SPIClass::transfer(void* tx, void* rx, size_t len)
{
    spi_.BlockingTransmitAndReceive(static_cast<uint8_t*>(tx),
                                    static_cast<uint8_t*>(rx),
                                    len,
                                    100);
}

void SPIClass::transfer(void* buf, size_t len)
{
    // For in-place transfer, use the same buffer for TX and RX
    spi_.BlockingTransmitAndReceive(static_cast<uint8_t*>(buf), 
                                   static_cast<uint8_t*>(buf),
                                   len,
                                   100);
}
