#include "imu.h"

// CS->CLK delay, MPU6000 - 8ns
// CS->CLK delay, ICM42688P - 39ns
constexpr uint32_t SETUP_TIME_NS  = 39;   // For ICM42688P
constexpr uint32_t HOLD_TIME_NS   = 18;   // For ICM42688P
constexpr uint8_t  READ_BIT_MASK  = 0x80; // High bit for read

using namespace uvos;

namespace uvos {

/* Default SPI frequency is 1 Mhz */
static uint8_t spi_freq_mhz = 1;

SpiHandle spi_hdl_;           // Handle we'll use to interact with IMU SPI
SpiHandle::Config spi_cfg_;   // Structure to configure the IMU SPI instance

// ICM SPI static init()
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

// ICM42688P imu CS pin (using software driven CS) static init()
GPIO csPin_local;

struct CsPinInit {
    CsPinInit() { csPin_local.Init(Pin(PORTC, 15), GPIO::Mode::OUTPUT, GPIO::Pull::PULLUP); }
};

static CsPinInit csPinInitializer;

#ifdef __cplusplus
extern "C" {
#endif

void inv_icm426xx_sleep_us(uint32_t us)
{
    System::DelayUs(us);    
}

static void inv_spi_chip_select_setup_delay(void)
{
    System::DelayNs(SETUP_TIME_NS);
}

static void inv_spi_chip_select_hold_time(void)
{
    System::DelayNs(HOLD_TIME_NS);
}

static void inv_spi_bus_select_device(void)
{
    csPin_local.Write(GPIO_PIN_RESET);
    inv_spi_chip_select_setup_delay();
}

static void inv_spi_bus_deselect_device(void)
{
    inv_spi_chip_select_hold_time();
    csPin_local.Write(GPIO_PIN_SET);
}

static uint8_t inv_spi_transfer_byte(uint8_t txByte)
{
    uint8_t rxByte = 0xFF;
    if (spi_hdl_.BlockingTransferLL(&txByte, &rxByte, 1) != SpiHandle::Result::OK) {
        return 0xFF; // some error indicator
    }
    return rxByte;
}

static int inv_spi_bus_read_registers(uint8_t addr, uint8_t count, uint8_t* data)
{
    inv_spi_bus_select_device();

    // bit7 = 1 indicates read.
    inv_spi_transfer_byte(addr | 0x80);

    if (spi_hdl_.BlockingTransferLL(nullptr, data, count) != SpiHandle::Result::OK) {
        inv_spi_bus_deselect_device();
        return -1; // or other error code
    }

    inv_spi_bus_deselect_device();

    return 0;
}

static int inv_spi_bus_write_registers(uint8_t addr, const uint8_t* data, uint8_t count)
{
    inv_spi_bus_select_device();

    // For write, bit7 = 0, so just ensure read bit is cleared:
    inv_spi_transfer_byte(addr & 0x7F);

    // Now write 'count' bytes in burst
    if (spi_hdl_.BlockingTransferLL(const_cast<uint8_t*>(data), nullptr, count) != SpiHandle::Result::OK) {
        inv_spi_bus_deselect_device();
        return -1; // or other error code
    }

    inv_spi_bus_deselect_device();
    return 0; // success
}

int inv_io_hal_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    return inv_spi_bus_read_registers(reg, static_cast<uint8_t>(rlen), rbuffer);
}

int inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t* wbuffer, uint32_t wlen)
{
    return inv_spi_bus_write_registers(reg, wbuffer, static_cast<uint8_t>(wlen));
}

static inline void spiSelect(GPIO_TypeDef *CS_Port, uint16_t CS_Pin)
{
    // Drive CS low
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

    // Min setup delay for IMU CS per datasheet
    System::DelayNs(SETUP_TIME_NS);
}

static inline void spiDeselect(GPIO_TypeDef *CS_Port, uint16_t CS_Pin)
{
    // Min hold time for IMU CS deselect per datasheet
    System::DelayNs(HOLD_TIME_NS);

    // Drive CS high
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);
}

static int spiReadBytes(SpiHandle& spi,
                        GPIO_TypeDef* cs_port,
                        uint16_t cs_pin,
                        uint8_t reg,
                        uint8_t *buf,
                        uint32_t len)
{
    // Enforce the maximum byte read
    if (len > IMU::IMU_MAX_READ) {
        return INV_ERROR_SIZE;
    }

    // Set the high (read) bit of the register address
    reg |= 0x80;

    spiSelect(cs_port, cs_pin);

    // First send the register address with high bit set
    spi.BlockingTransferLL(&reg, nullptr, 1);

    // Read 'len' bytes
    spi.BlockingTransferLL(nullptr, buf, len);

    spiDeselect(cs_port, cs_pin);

    return 0;
}

static int spiWriteBytes(SpiHandle& spi,
                         GPIO_TypeDef* cs_port,
                         uint16_t cs_pin,
                         uint8_t reg,
                         uint8_t *buf,
                         uint32_t len)
{
    // Enforce the maximum byte write
    if (len > IMU::IMU_MAX_WRITE) {
        return INV_ERROR_SIZE;
    }

    // Verify Bit7 = 0 for write
    reg &= 0x7F;

    spiSelect(cs_port, cs_pin);

    // First send the register address
    spi.BlockingTransferLL(&reg, nullptr, 1);

    // Write 'len' bytes
    spi.BlockingTransferLL(buf, nullptr, len);

    spiDeselect(cs_port, cs_pin);

    return 0;
}

#ifdef __cplusplus
}
#endif

//------------------------------------------------------------------------------
// Static callback pointer for user sensor events
//------------------------------------------------------------------------------
void (*IMU::userEventCb_)(inv_icm426xx_sensor_event_t *event) = nullptr;

//------------------------------------------------------------------------------
// Constructor: store the spi reference, capture chip-select pin from SPI config,
// and set up the TDK transport structure.
//------------------------------------------------------------------------------
IMU::IMU(SpiHandle &spi) : spi_(spi)
{
    // Retrieve the CS pin from the SPI config
    uvs_gpio_pin nss_pin = spi_.GetConfig().pin_config.nss;

    uvs_gpio_pin uvs_cs_pin = spi_.GetConfig().pin_config.nss;

    // Convert to GPIO::Pin
    Pin initPin(static_cast<uvos::GPIOPort>(nss_pin.port), nss_pin.pin);

    GPIO::Config config;
    config.pin = initPin;
    config.mode = uvos::GPIO::Mode::OUTPUT;
    config.pull = uvos::GPIO::Pull::NOPULL;
    config.speed = uvos::GPIO::Speed::LOW;
    csPin_.Init(config);

    // Save the CS pin port and pin number for quick access
    p_cs_port_ = uvs_hal_map_get_port(&uvs_cs_pin);
    cs_pin_ = uvs_hal_map_get_pin(&uvs_cs_pin);
}

//------------------------------------------------------------------------------
// Initialize the sensor via the TDK driver init function
//------------------------------------------------------------------------------
int IMU::Init()
{
    int rc = 0;
    uint8_t who_am_i;
    struct inv_icm426xx_serif imu_serif;

    // Initialize the TDK transport fields
    imu_serif.context   = this;
    imu_serif.read_reg  = &IMU::spiReadRegs;
    imu_serif.write_reg = &IMU::spiWriteRegs;

    // Set maximum read/write sizes (enforced by driver)
    imu_serif.max_read  = IMU_MAX_READ;
    imu_serif.max_write = IMU_MAX_WRITE;

    // Indicate SPI 4-wire interface
    imu_serif.serif_type = ICM426XX_UI_SPI4;

    // Give IMU some time to stabilize
    System::Delay(5);

    /* Initialize device */
    // INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx");

    // TDK driver will set up internal data, check device IDs, etc.
    rc = inv_icm426xx_init(&driver_, &imu_serif, &IMU::DriverEventCb);
    if (rc != INV_ERROR_SUCCESS) {
        // INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
        return rc;
    }

    /* Check WHOAMI */
    // INV_MSG(INV_MSG_LEVEL_INFO, "Check Icm426xx whoami value");

    rc = inv_icm426xx_get_who_am_i(&driver_, &who_am_i);
    if (rc != INV_ERROR_SUCCESS) {
        // INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Icm426xx whoami value.");
        return rc;
    }

    if (who_am_i != ICM_WHOAMI) {
        // INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)",
                // who_am_i, ICM_WHOAMI);
        return INV_ERROR;
    }

    return rc;
}

//------------------------------------------------------------------------------
// Perform a soft reset
//------------------------------------------------------------------------------
int IMU::Reset()
{
    return inv_icm426xx_device_reset(&driver_);
}

//------------------------------------------------------------------------------
// Enable/disable accelerometer
//------------------------------------------------------------------------------
int IMU::EnableAccelLNMode()
{
    return inv_icm426xx_enable_accel_low_noise_mode(&driver_);
}

int IMU::DisableAccel()
{
    return inv_icm426xx_disable_accel(&driver_);
}

//------------------------------------------------------------------------------
// Enable/disable gyroscope
//------------------------------------------------------------------------------
int IMU::EnableGyroLNMode()
{
    return inv_icm426xx_enable_gyro_low_noise_mode(&driver_);
}

int IMU::DisableGyro()
{
    return inv_icm426xx_disable_gyro(&driver_);
}

//------------------------------------------------------------------------------
// Configure accelerometer/gyro rates and full-scale ranges
//------------------------------------------------------------------------------
int IMU::SetAccelODR(ICM426XX_ACCEL_CONFIG0_ODR_t frequency)
{
    return inv_icm426xx_set_accel_frequency(&driver_, frequency);
}

int IMU::SetGyroODR(ICM426XX_GYRO_CONFIG0_ODR_t frequency)
{
    return inv_icm426xx_set_gyro_frequency(&driver_, frequency);
}

int IMU::SetAccelFSR(ICM426XX_ACCEL_CONFIG0_FS_SEL_t fsr)
{
    return inv_icm426xx_set_accel_fsr(&driver_, fsr);
}

int IMU::SetGyroFSR(ICM426XX_GYRO_CONFIG0_FS_SEL_t fsr)
{
    return inv_icm426xx_set_gyro_fsr(&driver_, fsr);
}

//------------------------------------------------------------------------------
// IMU self test and bias retrieval function
//------------------------------------------------------------------------------
int IMU::RunSelfTest(int* result, int* bias)
{
    int rc = 0;

    rc = inv_icm426xx_run_selftest(&driver_, result);

    if (rc == 0 && bias != nullptr) {
        rc = inv_icm426xx_get_st_bias(&driver_, bias);
    }

    return rc;
}

//------------------------------------------------------------------------------
// Read data from registers or FIFO
//------------------------------------------------------------------------------
int IMU::ReadDataFromRegisters()
{
    return inv_icm426xx_get_data_from_registers(&driver_);
}

int IMU::ReadDataFromFifo()
{
    return inv_icm426xx_get_data_from_fifo(&driver_);
}

//------------------------------------------------------------------------------
// Set the user callback for sensor events
//------------------------------------------------------------------------------
void IMU::SetSensorEventCallback(void (*userCb)(inv_icm426xx_sensor_event_t *event))
{
    userEventCb_ = userCb;
}

//------------------------------------------------------------------------------
// TDK driver event callback: forward to userEventCb_ if present
//------------------------------------------------------------------------------
void IMU::DriverEventCb(inv_icm426xx_sensor_event_t *event)
{
    if (userEventCb_) {
        userEventCb_(event);
    }
}

//------------------------------------------------------------------------------
// TDK transport layer callbacks for reading/writing registers
//------------------------------------------------------------------------------

/**
 * @brief  Called when TDK code needs to read 'len' bytes from 'reg'.
 *         Must match signature: int foo(struct inv_icm426xx_serif *serif, uint8_t reg,
 *                                       uint8_t *buf, uint32_t len);
 */
int IMU::spiReadRegs(struct inv_icm426xx_serif *serif,
                     uint8_t                    reg,
                     uint8_t                   *buf,
                     uint32_t                   len)
{
    if (!serif || !buf) {
        return INV_ERROR_BAD_ARG;
    }

    IMU *obj = reinterpret_cast<IMU*>(serif->context);
    if (!obj) {
        return INV_ERROR_BAD_ARG;
    }

    return spiReadBytes(obj->spi_,
                        obj->p_cs_port_,
                        obj->cs_pin_,
                        reg,
                        buf, len);
}

/**
 * @brief  Called when TDK code needs to write 'len' bytes to 'reg'.
 *         Must match signature: int foo(struct inv_icm426xx_serif *serif, uint8_t reg,
 *                                       const uint8_t *buf, uint32_t len);
 */
int IMU::spiWriteRegs(struct inv_icm426xx_serif * serif,
                      uint8_t                     reg,
                      const uint8_t*              buf,
                      uint32_t                    len)
{
    if (!serif || !buf) {
        return INV_ERROR_BAD_ARG;
    }

    IMU* obj = reinterpret_cast<IMU*>(serif->context);
    if (!obj) {
        return INV_ERROR_BAD_ARG;
    }

    return spiWriteBytes(obj->spi_,
                         obj->p_cs_port_,
                         obj->cs_pin_,
                         reg,
                         const_cast<uint8_t*>(buf), len);
}

} // namespace uvos
