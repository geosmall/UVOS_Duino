#include "imu.h"

constexpr uint32_t SETUP_TIME_NS  = 39;   // For ICM42688P
constexpr uint32_t HOLD_TIME_NS   = 18;   // For ICM42688P
constexpr uint8_t  READ_BIT_MASK  = 0x80; // High bit for read

using namespace uvos;

extern "C" void inv_icm426xx_sleep_us(uint32_t us)
{
    System::DelayUs(us);    
}

//------------------------------------------------------------------------------
// Inline helpers to select/deselect the chip, plus minimal delay for setup/hold.
// Adjust delays as needed for your hardware timing specifications.
//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------
// spiReadBytes() for reading 'len' bytes from 'reg'.
//------------------------------------------------------------------------------
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
    reg |= READ_BIT_MASK;

    spiSelect(cs_port, cs_pin);

    // First send the register address with high bit set
    spi.BlockingTransferLL(&reg, nullptr, 1);

    // Read 'len' bytes
    spi.BlockingTransferLL(nullptr, buf, len);

    spiDeselect(cs_port, cs_pin);

    return 0;
}

//------------------------------------------------------------------------------
// spiWriteBytes() for writing 'len' bytes to 'reg'.
//------------------------------------------------------------------------------
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

    spiSelect(cs_port, cs_pin);

    // First send the register address
    spi.BlockingTransferLL(&reg, nullptr, 1);

    // Write 'len' bytes
    spi.BlockingTransferLL(buf, nullptr, len);

    spiDeselect(cs_port, cs_pin);

    return 0;
}

namespace uvos {

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
    csPin_ = spi_.GetConfig().pin_config.nss;

    // Save the CS pin port and pin number for quick access
    p_cs_port_ = uvs_hal_map_get_port(&csPin_);
    cs_pin_ = uvs_hal_map_get_pin(&csPin_);
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
    imu_serif.configure = &IMU::spiConfigure;

    // Set maximum read/write sizes (enforced by driver)
    imu_serif.max_read  = IMU_MAX_READ;
    imu_serif.max_write = IMU_MAX_WRITE;

    // Indicate SPI 4-wire interface
    imu_serif.serif_type = ICM426XX_UI_SPI4;

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

    // Cast away const only for the SPI transfer function
    return spiWriteBytes(obj->spi_,
                         obj->p_cs_port_,
                         obj->cs_pin_,
                         reg,
                         const_cast<uint8_t*>(buf), len);
}

/**
 * @brief  Optional "configure" function pointer. Many systems leave this as no-op.
 *         Must match: int (*configure)(struct inv_icm426xx_serif *serif);
 */
int IMU::spiConfigure(struct inv_icm426xx_serif *serif)
{
    (void)serif;
    // No special SPI reconfiguration required in most systems
    return 0;
}

} // namespace uvos
