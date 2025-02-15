/* --------------------------------------------------------------------------------------
 *  Extern functions definition - Invensense to UVOS adapters
 * -------------------------------------------------------------------------------------- */

#include "per/util/spi_util.h"
#include "stm32h7xx_ll_spi.h"

/* Default SPI frequency is 1 Mhz */
static uint8_t spi_default_freq_mhz_ = 1;
// static GPIO_TypeDef* CS_GPIO_Port;
// static uint32_t CS_GPIO_Pin;
static uint32_t disable_delay_;
static SPI_TypeDef* SpiInstancePtr_; // Pointer to SPI instance

// ICM42688P imu CS pin (using software driven CS)
GPIO csPin_;

#ifdef __cplusplus
extern "C" {
#endif

void inv_board_hal_init(void);
void inv_spi_chip_select_setup_delay(void);
void inv_spi_chip_select_hold_time(void);
void inv_spi_bus_select_device(void);
void inv_spi_bus_deselect_device(void);
int inv_spi_transfer(SPI_TypeDef *instance, uint8_t *rxData, const uint8_t *txData, int len);
uint8_t inv_spi_transfer_byte(SPI_TypeDef *instance, uint8_t txByte);
int inv_spi_bus_read_registers(uint8_t addr, uint8_t count, uint8_t* data);
int inv_spi_bus_write_register(uint8_t reg, const uint8_t* data);


/******************************************************/
/* Low-level serial interface function implementation */
/******************************************************/

void inv_io_hal_board_init(void)
{
    // Initialize LED0, turned off
    hw.SetLed(off);
}

void inv_io_hal_configure_spi_speed(uint8_t spi_freq_mhz)
{
    spi_default_freq_mhz_ = spi_freq_mhz;
}

int inv_io_hal_init(struct inv_icm426xx_serif* serif)
{
    uint8_t spi_freq_mhz = spi_default_freq_mhz_;

    // Initialize chip select (CS) pin
    csPin_.Init(CS_PIN, GPIO::Mode::OUTPUT, GPIO::Pull::PULLUP);

    // Configure the Uart Peripheral to print out results
    UartHandler::Config uart_conf;
    uart_conf.periph        = UART_NUM;
    uart_conf.mode          = UartHandler::Config::Mode::TX;
    uart_conf.pin_config.tx = TX_PIN;
    uart_conf.pin_config.rx = RX_PIN;

    // Initialize the uart peripheral and start the DMA transmit
    uart.Init(uart_conf);

    // Configure the ICM-42688P IMU SPI interface (match for Matek_H743 WLITE)
    spi_conf.periph = SpiHandle::Config::Peripheral::SPI_1;
    spi_conf.mode = SpiHandle::Config::Mode::MASTER;
    spi_conf.direction = SpiHandle::Config::Direction::TWO_LINES;
    spi_conf.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    spi_conf.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;

#ifdef USE_SOFT_NSS
    spi_conf.nss = SpiHandle::Config::NSS::SOFT;
#else
    spi_conf.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
#endif /* USE_SOFT_NSS */

    spi_conf.pin_config.nss = CS_PIN;
    spi_conf.pin_config.sclk = SCLK_PIN;
    spi_conf.pin_config.miso = MISO_PIN;
    spi_conf.pin_config.mosi = MOSI_PIN;

    // spi_conf.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_32;
    spi_handle.GetBaudHz(spi_conf.periph, (spi_freq_mhz * 1'000'000), spi_conf.baud_prescaler);

    // Initialize the IMU SPI instance
    spi_handle.Init(spi_conf);

    // Save away the SPI instance for later use
    SpiInstancePtr_ = SpiHandle::PeripheralToHAL(spi_conf.periph);

    // Compute delay used below in inv_spi_transfer()
    disable_delay_ = spi_compute_disable_delay_us(SpiInstancePtr_);

    return 0;
}

int inv_io_hal_configure(struct inv_icm426xx_serif *serif)
{
    switch (serif->serif_type) {
    default:
        return -1;
    }
}

int inv_io_hal_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    // return inv_spi_master_read_register(INV_SPI_AP, reg, rlen, rbuffer);
    return inv_spi_bus_read_registers(reg, (uint8_t)rlen, rbuffer);
}

int inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t* wbuffer, uint32_t wlen)
{
    int rc;

    for (uint32_t i = 0; i < wlen; i++) {
        // rc = inv_spi_master_write_register(INV_SPI_AP, reg + i, 1, &wbuffer[i]);
        rc = inv_spi_bus_write_register(reg + i, &wbuffer[i]);
        if (rc) {
            return rc;
        }
    }
    return 0;
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

int inv_spi_transfer(SPI_TypeDef *instance, uint8_t *rxData, const uint8_t *txData, int len)
{
    LL_SPI_SetTransferSize(instance, len);
    LL_SPI_Enable(instance);
    LL_SPI_StartMasterTransfer(instance);
    while (len) {
        int spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_TXP(instance)) {
            if ((spiTimeout--) == 0) {
                // spiTimeoutUserCallback(instance); gls
                return -1;
            }
        }
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);

        spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_RXP(instance)) {
            if ((spiTimeout--) == 0) {
                // spiTimeoutUserCallback(instance); gls
                return -1;
            }
        }
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }
    while (!LL_SPI_IsActiveFlag_EOT(instance));
    // Add a delay before disabling SPI otherwise last-bit/last-clock may be truncated
    // See https://github.com/stm32duino/Arduino_Core_STM32/issues/1294
    // Computed delay is half SPI clock
    System::DelayUs(disable_delay_);

    LL_SPI_ClearFlag_TXTF(instance);
    LL_SPI_Disable(instance);

    return 0;
}

uint8_t inv_spi_transfer_byte(SPI_TypeDef *instance, uint8_t txByte)
{
    uint8_t value = 0xFF;
    if (!inv_spi_transfer(instance, &value, &txByte, 1)) {
        return 0xFF;
    }
    return value;
}

int inv_spi_bus_read_registers(uint8_t addr, uint8_t count, uint8_t* data)
{
    SPI_TypeDef* instance = SpiInstancePtr_;

    inv_spi_bus_select_device();

    inv_spi_transfer_byte(instance, (addr | 0x80));
    for (uint8_t i = 0; i < count; i++) {
        inv_spi_transfer(instance, &data[i], NULL, 1);
    }

    inv_spi_bus_deselect_device();

    return 0;
}

int inv_spi_bus_write_register(uint8_t reg, const uint8_t* data)
{
    SPI_TypeDef* instance = SpiInstancePtr_;

    inv_spi_bus_select_device();

    inv_spi_transfer_byte(instance, reg);
    inv_spi_transfer_byte(instance, *data);

    inv_spi_bus_deselect_device();

    return 0;
}

#ifdef __cplusplus
}
#endif