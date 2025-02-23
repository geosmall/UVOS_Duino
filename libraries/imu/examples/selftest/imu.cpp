#include "imu.h"

constexpr uint32_t SETUP_TIME_NS  = 39;   // For ICM42688P
constexpr uint32_t HOLD_TIME_NS   = 18;   // For ICM42688P
constexpr uint8_t  READ_BIT_MASK  = 0x80; // High bit for read

using namespace uvos;

namespace uvos {

/* Default SPI frequency is 1 Mhz */
static uint8_t spi_freq_mhz = 1;

SpiHandle spi_hdl_;           // Handle we'll use to interact with IMU SPI
SpiHandle::Config spi_cfg_;   // Structure to configure the IMU SPI instance

struct spiHdlInit {
    spiHdlInit() {
	    // Configure the ICM-42688P IMU SPI interface (match for Matek_H743 WLITE)
	    spi_cfg_.periph = SpiHandle::Config::Peripheral::SPI_1;
	    spi_cfg_.mode = SpiHandle::Config::Mode::MASTER;
	    spi_cfg_.direction = SpiHandle::Config::Direction::TWO_LINES;
	    spi_cfg_.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
	    spi_cfg_.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;
    	spi_cfg_.nss = SpiHandle::Config::NSS::SOFT;

	    spi_cfg_.pin_config.nss = Pin(PORTC, 15);
	    spi_cfg_.pin_config.sclk = Pin(PORTA, 5);
	    spi_cfg_.pin_config.miso = Pin(PORTA, 6);
	    spi_cfg_.pin_config.mosi = Pin(PORTD, 7);

	    spi_hdl_.GetBaudHz(spi_cfg_.periph, (spi_freq_mhz * 1'000'000), spi_cfg_.baud_prescaler);

	    // Initialize the IMU SPI instance
	    spi_hdl_.Init(spi_cfg_);
    }
};

static spiHdlInit spiHdlInitializer;

// ICM42688P imu CS pin (using software driven CS)
GPIO csPin_;

struct CsPinInit {
    CsPinInit() { csPin_.Init(Pin(PORTC, 15), GPIO::Mode::OUTPUT, GPIO::Pull::PULLUP); }
};

static CsPinInit csPinInitializer;

#ifdef __cplusplus
extern "C" {
#endif

extern "C" void inv_icm426xx_sleep_us(uint32_t us)
{
    System::DelayUs(us);    
}

void inv_spi_chip_select_setup_delay(void)
{
    // CS->CLK delay, MPU6000 - 8ns
    // CS->CLK delay, ICM42688P - 39ns
    System::DelayNs(39);
}

void inv_spi_chip_select_hold_time(void)
{
    // CLK->CS delay, MPU6000 - 500ns
    // CS->CLK delay, ICM42688P - 18ns
    System::DelayNs(18);
}

void inv_spi_bus_select_device(void)
{
    csPin_.Write(GPIO_PIN_RESET);
    inv_spi_chip_select_setup_delay();
}

void inv_spi_bus_deselect_device(void)
{
    inv_spi_chip_select_hold_time();
    csPin_.Write(GPIO_PIN_SET);
}

uint8_t inv_spi_transfer_byte(uint8_t txByte)
{
    uint8_t value = 0xFF;
    if (spi_hdl_.BlockingTransferLL(&txByte, &value, 1) != SpiHandle::Result::OK) {
        return 0xFF;
    }
    return value;
}

int inv_spi_bus_read_registers(uint8_t addr, uint8_t count, uint8_t* data)
{
    inv_spi_bus_select_device();

    inv_spi_transfer_byte((addr | 0x80));
    for (uint8_t i = 0; i < count; i++) {
        spi_hdl_.BlockingTransferLL(NULL, &data[i], 1);
    }

    inv_spi_bus_deselect_device();

    return 0;
}

int inv_spi_bus_write_register(uint8_t reg, const uint8_t* data)
{
    inv_spi_bus_select_device();

    inv_spi_transfer_byte(reg);
    inv_spi_transfer_byte(*data);

    inv_spi_bus_deselect_device();

    return 0;
}

int inv_io_hal_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    return inv_spi_bus_read_registers(reg, (uint8_t)rlen, rbuffer);
}

int inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t* wbuffer, uint32_t wlen)
{
    int rc;

    for (uint32_t i = 0; i < wlen; i++) {
        rc = inv_spi_bus_write_register(reg + i, &wbuffer[i]);
        if (rc) {
            return rc;
        }
    }
    return 0;
}

#ifdef __cplusplus
}
#endif

} // namespace uvos
