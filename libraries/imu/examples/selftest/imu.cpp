#include "imu.h"

// CS->CLK delay, MPU6000 - 8ns
// CS->CLK delay, ICM42688P - 39ns
constexpr uint32_t SETUP_TIME_NS_LOCAL  = 39;   // For ICM42688P
constexpr uint32_t HOLD_TIME_NS_LOCAL   = 18;   // For ICM42688P

using namespace uvos;

namespace uvos {

//------------------------------------------------------------------------------
// External delay function used by TDK library
//------------------------------------------------------------------------------
extern "C" void inv_icm426xx_sleep_us(uint32_t us)
{
    System::DelayUs(us);    
}

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
// Private SPI helper functions
//------------------------------------------------------------------------------

void IMU::SelectDevice()
{
    csPin_.Write(GPIO_PIN_RESET);
    System::DelayNs(SETUP_TIME_NS);
}

void IMU::DeselectDevice()
{
    System::DelayNs(HOLD_TIME_NS);
    csPin_.Write(GPIO_PIN_SET);
}

//------------------------------------------------------------------------------
// TDK transport layer callbacks for reading/writing registers
//------------------------------------------------------------------------------

/**
 * @brief  Called when TDK code needs to read 'len' bytes from 'reg'.
 *         Must match signature:
 *         int (*read_reg)(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
 */
int IMU::spiReadRegs(struct inv_icm426xx_serif *serif,
                     uint8_t                    reg,
                     uint8_t                   *buf,
                     uint32_t                   len)
{
    if (!serif || !buf) {
        return -1;
    }

    IMU *obj = reinterpret_cast<IMU*>(serif->context);
    if (!obj) {
        return -1;
    }

    // Enforce the maximum byte read
    if (len > IMU::IMU_MAX_READ) {
        return -1;
    }

    // Set the high (read) bit of the register address
    reg |= 0x80;

    obj->SelectDevice();

    // First send the register address with high bit set
    if (obj->spi_.BlockingTransferLL(&reg, nullptr, 1) != SpiHandle::Result::OK)
    {
        return -1;
    }

    // Read 'len' bytes
    if (obj->spi_.BlockingTransferLL(nullptr, const_cast<uint8_t*>(buf), len) != SpiHandle::Result::OK)
    {
        return -1;
    }

    obj->DeselectDevice();

    return 0;
}

/**
 * @brief  Called when TDK code needs to write 'len' bytes to 'reg'.
 *         Must match signature:
 *         int (*write_reg)(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *buf, uint32_t len)
 */
int IMU::spiWriteRegs(struct inv_icm426xx_serif * serif,
                      uint8_t                     reg,
                      const uint8_t*              buf,
                      uint32_t                    len)
{
    if (!serif || !buf)
    {
        return -1;
    }

    // Retrieve pointer to IMU object from serial interface context
    IMU* obj = reinterpret_cast<IMU*>(serif->context);
    if (!obj)
    {
        return -1;
    }

    // Enforce the maximum byte write
    if (len > IMU::IMU_MAX_WRITE)
    {
        return -1;
    }

    // Verify Bit7 = 0 for write
    reg &= 0x7F;

    obj->SelectDevice();

    // First send the register address
    if (obj->spi_.BlockingTransferLL(&reg, nullptr, 1) != SpiHandle::Result::OK)
    {
        return -1;
    }

    // Write 'len' bytes
    if (obj->spi_.BlockingTransferLL(const_cast<uint8_t*>(buf), nullptr, len)  != SpiHandle::Result::OK)
    {
        return -1;
    }

    obj->DeselectDevice();

    return 0;
}

} // namespace uvos
