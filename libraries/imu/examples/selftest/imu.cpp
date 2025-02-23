#include "imu.h"

constexpr uint32_t SETUP_TIME_NS  = 39;   // For ICM42688P
constexpr uint32_t HOLD_TIME_NS   = 18;   // For ICM42688P
constexpr uint8_t  READ_BIT_MASK  = 0x80; // High bit for read

using namespace uvos;

namespace uvos {

/* Default SPI frequency is 1 Mhz */
static uint8_t spi_default_freq_mhz_ = 1;

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
    // csPin_.Init(Pin(PORTC, 15), GPIO::Mode::OUTPUT, GPIO::Pull::PULLUP);

    csPin_.Write(GPIO_PIN_RESET);
    inv_spi_chip_select_setup_delay();
}

void inv_spi_bus_deselect_device(void)
{
    // csPin_.Init(Pin(PORTC, 15), GPIO::Mode::OUTPUT, GPIO::Pull::PULLUP);

    inv_spi_chip_select_hold_time();
    csPin_.Write(GPIO_PIN_SET);
}

// int inv_io_hal_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
// {
//     return inv_spi_bus_read_registers(reg, (uint8_t)rlen, rbuffer);
// }

// int inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t* wbuffer, uint32_t wlen)
// {
//     int rc;

//     for (uint32_t i = 0; i < wlen; i++) {
//         rc = inv_spi_bus_write_register(reg + i, &wbuffer[i]);
//         if (rc) {
//             return rc;
//         }
//     }
//     return 0;
// }

#ifdef __cplusplus
}
#endif

} // namespace uvos
