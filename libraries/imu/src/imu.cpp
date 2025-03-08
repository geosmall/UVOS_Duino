#include "imu.h"

// CS->CLK delay, MPU6000 - 8ns
// CS->CLK delay, ICM42688P - 39ns
constexpr uint32_t SETUP_TIME_NS_LOCAL  = 39;   // For ICM42688P
constexpr uint32_t HOLD_TIME_NS_LOCAL   = 18;   // For ICM42688P

using namespace uvos;

namespace uvos {

//------------------------------------------------------------------------------
// External time functions used by TDK library
//------------------------------------------------------------------------------
extern "C" void inv_icm426xx_sleep_us(uint32_t us)
{
    System::DelayUs(us);    
}

extern "C" uint64_t inv_icm426xx_get_time_us(void)
{
    System::GetUs();
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

    // Configure CS pin for output w/ pullup
    GPIO::Config config;
    config.pin = initPin;
    config.mode = uvos::GPIO::Mode::OUTPUT;
    config.pull = uvos::GPIO::Pull::PULLUP;
    config.speed = uvos::GPIO::Speed::MEDIUM;
    csPin_.Init(config);
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

    // Config TDK driver for SPI 4-wire interface
    imu_serif.serif_type = ICM426XX_UI_SPI4;

    // Give IMU some time to stabilize
    System::Delay(5);

    // Initialize device (exits with PWR_MGMT0 = 0)
    rc = inv_icm426xx_init(&driver_, &imu_serif, &IMU::DriverEventCb);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }

    // Set to Bank 0 for reminder of Init
    // Turn off ACC and GYRO so they can be configured
    // See section 12.9 in ICM-42688-P datasheet v1.8
    // rc |= inv_icm426xx_set_reg_bank(&driver_, 0);
    // rc |= SetPwrState(PwrState::POWER_OFF);
    // if (rc != INV_ERROR_SUCCESS) {
    //     return rc;
    // }

    // Config INT_CONFIG1 as int pulse 8 µs, disable de-assert duration, leave async off (disabled)
    uint8_t int_config1_val;
    rc |= inv_icm426xx_read_reg(&driver_, MPUREG_INT_CONFIG1, 1, &int_config1_val);
    int_config1_val |= (ICM426XX_INT_TPULSE_DURATION_8_US | ICM426XX_INT_TDEASSERT_DISABLED | ICM426XX_INT_CONFIG1_ASY_RST_DISABLED);
    rc |= inv_icm426xx_write_reg(&driver_, MPUREG_INT_CONFIG1, 1, &int_config1_val);

    // Disable all INT1 sources
    inv_icm426xx_interrupt_parameter_t config_int1 = {INV_ICM426XX_DISABLE};
    rc |= inv_icm426xx_set_config_int1(&driver_, &config_int1);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }

    System::Delay(15);

    // Disable fifo usage, data will be read from sensor registers, and
    // int1 is configured on Data Ready by inv_icm426xx_configure_fifo()
    rc |= inv_icm426xx_configure_fifo(&driver_, INV_ICM426XX_FIFO_DISABLED);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }

    // Turn ACC and GYRO back on
    // rc |= SetPwrState(PwrState::POWER_ON);
    // if (rc != INV_ERROR_SUCCESS) {
    //     return rc;
    // }

    // Retrieve device ID
    rc = inv_icm426xx_get_who_am_i(&driver_, &who_am_i);
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }

    // Verify expected ID
    if (who_am_i != ICM_WHOAMI) {
        return INV_ERROR;
    }

    return rc;
}

// Undocumented feature "AFSR", auto-switches between low-noise low range and
// a high range, when it switches it glitches.  See below.
#define INTF_CONFIG1_AFSR_MASK 0xC0
#define INTF_CONFIG1_AFSR_DISABLE 0x40

//------------------------------------------------------------------------------
// Configure the device full scales and output frequencies
//------------------------------------------------------------------------------
int IMU::ConfigureInvDevice(AccelFS acc_fsr_g, GyroFS gyr_fsr_dps, AccelODR acc_freq, GyroODR gyr_freq)
{
    int rc = 0;

    rc |= inv_icm426xx_enable_clkin_rtc(&driver_, 0);

    // Fix for stalls in gyro output. See GitHub
    // https://github.com/ArduPilot/ardupilot/pull/25332
    // https://github.com/betaflight/betaflight/issues/12970
    uint8_t intf_config1_val;
    rc |= inv_icm426xx_set_reg_bank(&driver_, 0);
    rc |= inv_icm426xx_read_reg(&driver_, MPUREG_INTF_CONFIG1, 1, &intf_config1_val);
    intf_config1_val &= ~INTF_CONFIG1_AFSR_MASK;
    intf_config1_val |= INTF_CONFIG1_AFSR_DISABLE;
    rc |= inv_icm426xx_write_reg(&driver_, MPUREG_INTF_CONFIG1, 1, &intf_config1_val);

    rc |= inv_icm426xx_set_accel_fsr(&driver_, static_cast<ICM426XX_ACCEL_CONFIG0_FS_SEL_t>(acc_fsr_g));
    rc |= inv_icm426xx_set_gyro_fsr(&driver_, static_cast<ICM426XX_GYRO_CONFIG0_FS_SEL_t>(gyr_fsr_dps));

    rc |= inv_icm426xx_set_accel_frequency(&driver_, static_cast<ICM426XX_ACCEL_CONFIG0_ODR_t>(acc_freq));
    rc |= inv_icm426xx_set_gyro_frequency(&driver_, static_cast<ICM426XX_GYRO_CONFIG0_ODR_t>(gyr_freq));

    rc |= inv_icm426xx_enable_accel_low_noise_mode(&driver_);
    rc |= inv_icm426xx_enable_gyro_low_noise_mode(&driver_);

    /* Wait Max of ICM426XX_GYR_STARTUP_TIME_US and ICM426XX_ACC_STARTUP_TIME_US*/
    (ICM426XX_GYR_STARTUP_TIME_US > ICM426XX_ACC_STARTUP_TIME_US) ?
    inv_icm426xx_sleep_us(ICM426XX_GYR_STARTUP_TIME_US) :
    inv_icm426xx_sleep_us(ICM426XX_ACC_STARTUP_TIME_US);

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
int IMU::SetAccelODR(AccelODR freq)
{
    return inv_icm426xx_set_accel_frequency(&driver_, static_cast<ICM426XX_ACCEL_CONFIG0_ODR_t>(freq));
}

int IMU::SetGyroODR(GyroODR freq)
{
    return inv_icm426xx_set_gyro_frequency(&driver_, static_cast<ICM426XX_GYRO_CONFIG0_ODR_t>(freq));
}

int IMU::SetAccelFSR(AccelFS fsr)
{
    return inv_icm426xx_set_accel_fsr(&driver_, static_cast<ICM426XX_ACCEL_CONFIG0_FS_SEL_t>(fsr));
}

int IMU::SetGyroFSR(GyroFS fsr)
{
    return inv_icm426xx_set_gyro_fsr(&driver_, static_cast<ICM426XX_GYRO_CONFIG0_FS_SEL_t>(fsr));
}

int IMU::SetPwrState(PwrState state)
{
    int rc = 0;

    /* Per DK-42688-P_SmartMotion_eMD, powering the gyroscope on immediately after powering it off
     * can cause device failure. The gyroscope proof mass can continue vibrating after it has been powered
     * off, and powering it back on immediately can result in unpredictable proof mass movement. After
     * powering the gyroscope off, a period of > 150ms should be allowed to elapse before it is powered back
     * on. */
    System::Delay(150);

    rc |= inv_icm426xx_set_reg_bank(&driver_, 0);

    uint8_t pwr_reg = 0;
    if (state == PwrState::POWER_OFF)
    {
        rc |= inv_icm426xx_write_reg(&driver_, MPUREG_PWR_MGMT_0, 1, &pwr_reg);
    } else {
        pwr_reg = ((uint8_t)ICM426XX_PWR_MGMT_0_TEMP_EN |
                   (uint8_t)ICM426XX_PWR_MGMT_0_GYRO_MODE_LN |
                   (uint8_t)ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN);
        rc |= inv_icm426xx_write_reg(&driver_, MPUREG_PWR_MGMT_0, 1, &pwr_reg);
    }
    if (rc != INV_ERROR_SUCCESS) {
        return rc;
    }

    System::Delay(10);

    return rc;
}

//------------------------------------------------------------------------------
// Data Ready interrupt on INT1 pin.
//------------------------------------------------------------------------------
int IMU::EnableDataReadyInt1()
{
    int rc = 0;

    inv_icm426xx_interrupt_parameter_t config_int1;
    rc |= inv_icm426xx_get_config_int1(&driver_, &config_int1);

    if (config_int1.INV_ICM426XX_UI_DRDY == INV_ICM426XX_DISABLE) {
        config_int1.INV_ICM426XX_UI_DRDY = INV_ICM426XX_ENABLE;
        rc |= inv_icm426xx_set_config_int1(&driver_, &config_int1);
    }
    return rc;
}

int IMU::DisableDataReadyInt1()
{
    int rc = 0;

    inv_icm426xx_interrupt_parameter_t config_int1;
    rc |= inv_icm426xx_get_config_int1(&driver_, &config_int1);

    if (config_int1.INV_ICM426XX_UI_DRDY == INV_ICM426XX_ENABLE) {
        config_int1.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
        rc |= inv_icm426xx_set_config_int1(&driver_, &config_int1);
    }
    return rc;
}

//------------------------------------------------------------------------------
// IMU self test and bias retrieval function
//------------------------------------------------------------------------------
int IMU::RunSelfTest(int* result, std::array<int, 6>* bias)
{
    int rc = 0;

    rc = inv_icm426xx_run_selftest(&driver_, result);

    if (rc == 0 && bias != nullptr) {
        rc = inv_icm426xx_get_st_bias(&driver_, bias->data());
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

// int IMU::ReadIMU6(uint8_t* buf)
int IMU::ReadIMU6(std::array<uint8_t, 6>& buf)
{
    int rc = 0;
    uint8_t int_status;

    // First field of driver_ (struct inv_icm426xx) is struct inv_icm426xx_transport object.
    // Therefore, cast address of driver to struct inv_icm426xx_transport to access serif.
    // struct inv_icm426xx_transport* t = (struct inv_icm426xx_transport*) &driver_;
    struct inv_icm426xx_transport* t = reinterpret_cast<struct inv_icm426xx_transport*>(&driver_);

    /* Ensure data ready status bit is set */
    rc |= inv_icm426xx_read_reg(&driver_, MPUREG_INT_STATUS, 1, &int_status);
    if (rc) { return rc; }

    if (int_status & BIT_INT_STATUS_DRDY) {
        rc |= t->serif.read_reg(&(t->serif), MPUREG_ACCEL_DATA_X0_UI, buf.data(), buf.size());
    }

    return rc;
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
int IMU::spiReadRegs(struct inv_icm426xx_serif* serif,
                     uint8_t                    reg,
                     uint8_t*                   buf,
                     uint32_t                   len)
{
    if (!serif || !buf) {
        return -1;
    }

    IMU* obj = reinterpret_cast<IMU*>(serif->context);
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
    if (obj->spi_.BlockingTransferLL(&reg, nullptr, 1) != SpiHandle::Result::OK) {
        return -1;
    }

    // Read 'len' bytes
    if (obj->spi_.BlockingTransferLL(nullptr, const_cast<uint8_t*>(buf), len) != SpiHandle::Result::OK) {
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
    if (!serif || !buf) {
        return -1;
    }

    // Retrieve pointer to IMU object from serial interface context
    IMU* obj = reinterpret_cast<IMU*>(serif->context);
    if (!obj) {
        return -1;
    }

    // Enforce the maximum byte write
    if (len > IMU::IMU_MAX_WRITE) {
        return -1;
    }

    // Verify Bit7 = 0 for write
    reg &= 0x7F;

    obj->SelectDevice();

    // First send the register address
    if (obj->spi_.BlockingTransferLL(&reg, nullptr, 1) != SpiHandle::Result::OK) {
        return -1;
    }

    // Write 'len' bytes
    if (obj->spi_.BlockingTransferLL(const_cast<uint8_t*>(buf), nullptr, len)  != SpiHandle::Result::OK) {
        return -1;
    }

    obj->DeselectDevice();

    return 0;
}

} // namespace uvos
