#include "imu.h"

// CS->CLK delay, MPU6000 - 8ns
// CS->CLK delay, ICM42688P - 39ns
constexpr uint32_t SETUP_TIME_NS_LOCAL  = 39;   // For ICM42688P
constexpr uint32_t HOLD_TIME_NS_LOCAL   = 18;   // For ICM42688P

using namespace uvos;

namespace uvos {

/* Default implementation converts ICM endian to little endian */
static void inv_icm426xx_format_data(const uint8_t endian, const uint8_t *in, uint16_t *out)
{
    if (endian == ICM426XX_INTF_CONFIG0_DATA_BIG_ENDIAN)
        *out = (in[0] << 8) | in[1];
    else
        *out = (in[1] << 8) | in[0];
}

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
// Constructor: empty
//------------------------------------------------------------------------------
IMU::IMU() : p_spi_(nullptr) {}

//------------------------------------------------------------------------------
// Store the spi reference, capture chip-select pin from SPI config,
// set up the TDK transport structure and Initialize the sensor via
// the TDK driver init function.
//------------------------------------------------------------------------------
IMU::Result IMU::Init(SpiHandle& spi)
{
    int rc = 0;
    uint8_t who_am_i;
    struct inv_icm426xx_serif imu_serif;

    // Save the handle to the spi object
    if (&spi != nullptr) {
        p_spi_ = &spi;
        initialized_ = true;
    } else {
        initialized_ = false;
        return Result::ERR;
    }

    // Retrieve the CS pin from the SPI config
    uvs_gpio_pin nss_pin = p_spi_->GetConfig().pin_config.nss;

    // Convert to GPIO::Pin
    Pin initPin(static_cast<uvos::GPIOPort>(nss_pin.port), nss_pin.pin);

    // Configure CS pin for output w/ pullup
    GPIO::Config config;
    config.pin = initPin;
    config.mode = uvos::GPIO::Mode::OUTPUT;
    config.pull = uvos::GPIO::Pull::PULLUP;
    config.speed = uvos::GPIO::Speed::MEDIUM;
    csPin_.Init(config);

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
        return Result::ERR;
    }

    // Config INT_CONFIG1 as int pulse 8 µs, disable de-assert duration, leave async off (disabled)
    uint8_t int_config1_val;
    rc |= inv_icm426xx_read_reg(&driver_, MPUREG_INT_CONFIG1, 1, &int_config1_val);
    int_config1_val |= (ICM426XX_INT_TPULSE_DURATION_8_US | ICM426XX_INT_TDEASSERT_DISABLED | ICM426XX_INT_CONFIG1_ASY_RST_DISABLED);
    rc |= inv_icm426xx_write_reg(&driver_, MPUREG_INT_CONFIG1, 1, &int_config1_val);

    // Disable all INT1 sources
    inv_icm426xx_interrupt_parameter_t config_int1 = {INV_ICM426XX_DISABLE};
    rc |= inv_icm426xx_set_config_int1(&driver_, &config_int1);
    if (rc != INV_ERROR_SUCCESS) {
        return Result::ERR;
    }

    System::Delay(15);

    // Disable fifo usage, data will be read from sensor registers, and
    // Int1 is configured for Data Ready by inv_icm426xx_configure_fifo()
    rc |= inv_icm426xx_configure_fifo(&driver_, INV_ICM426XX_FIFO_DISABLED);
    if (rc != INV_ERROR_SUCCESS) {
        return Result::ERR;
    }

    // Retrieve device ID
    rc = inv_icm426xx_get_who_am_i(&driver_, &who_am_i);
    if (rc != INV_ERROR_SUCCESS) {
        return Result::ERR;
    }

    // Verify expected ID
    if (who_am_i != ICM_WHOAMI) {
        return Result::ERR;
    }

    return Result::OK;
}

// Undocumented feature "AFSR", auto-switches between low-noise low range and
// a high range, when it switches it glitches.  See below.
#define INTF_CONFIG1_AFSR_MASK 0xC0
#define INTF_CONFIG1_AFSR_DISABLE 0x40

//------------------------------------------------------------------------------
// Configure the device full scales and output frequencies
//------------------------------------------------------------------------------
IMU::Result IMU::ConfigureInvDevice(AccelFS acc_fsr_g, GyroFS gyr_fsr_dps, AccelODR acc_freq, GyroODR gyr_freq)
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

    return (rc == 0) ? Result::OK : Result::ERR;
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
    // Write FSR to hardware
    int rc = inv_icm426xx_set_accel_fsr(&driver_,
    static_cast<ICM426XX_ACCEL_CONFIG0_FS_SEL_t>(fsr));

    if (rc != 0) {
        accel_sensitivity_ = -1.0f;
        return rc;
    }

    // Update sensitivity (LSB/g) based on the FSR
    switch (fsr) {
    case gpm16:
        accel_sensitivity_ = 2048.0f; // ±16 G's
        break;
    case gpm8:
        accel_sensitivity_ = 4096.0f; // ±8 G's
        break;
    case gpm4:
        accel_sensitivity_ = 8192.0f; // ±4 G's
        break;
    case gpm2:
        accel_sensitivity_ = 16384.0f; // ±2 G's
        break;
    default:
        accel_sensitivity_ = -1.0f; // Should not occur
        break;
    }

    return rc;
}

int IMU::SetGyroFSR(GyroFS fsr)
{
    // Write FSR to hardware
    int rc = inv_icm426xx_set_gyro_fsr(&driver_,
                                       static_cast<ICM426XX_GYRO_CONFIG0_FS_SEL_t>(fsr));

    if (rc != 0) {
        gyro_sensitivity_ = -1.0f;
        return rc;
    }

    // Update sensitivity (LSB/dps) based on FSR
    switch (fsr) {
    case dps2000:
        gyro_sensitivity_ = 164.0f;   // ±2000 dps
        break;
    case dps1000:
        gyro_sensitivity_ = 328.0f;   // ±1000 dps
        break;
    case dps500:
        gyro_sensitivity_ = 655.0f;   // ±500 dps
        break;
    case dps250:
        gyro_sensitivity_ = 1311.0f;  // ±250 dps
        break;
    default:
        gyro_sensitivity_ = -1.0f; // Should not occur
        break;
    }

    return rc;
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

int IMU::ReadIMU6(std::array<int16_t, 6>& buf)
{
    int rc = 0;
    uint8_t int_status;
    uint8_t data[NUM_DATA_BYTES];

    // struct inv_icm426xx *s = &driver_;

    /* Ensure data ready status bit is set */
    rc |= inv_icm426xx_read_reg(&driver_, MPUREG_INT_STATUS, 1, &int_status);
    if (rc) {
        return rc;
    }

    if (int_status & BIT_INT_STATUS_DRDY) {
        rc |= inv_icm426xx_read_reg(&driver_, MPUREG_ACCEL_DATA_X0_UI, NUM_DATA_BYTES, data);
        buf[0] = (data[0] << 8) | data[1];
        buf[1] = (data[2] << 8) | data[3];
        buf[2] = (data[4] << 8) | data[5];
        buf[3] = (data[6] << 8) | data[7];
        buf[4] = (data[8] << 8) | data[9];
        buf[5] = (data[10] << 8) | data[11];
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

    // Verify object has been properly initialized with a pointer to spi obj
    if (!obj->initialized_ || (obj->p_spi_ == nullptr)) {
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
    if (obj->p_spi_->BlockingTransferLL(&reg, nullptr, 1) != SpiHandle::Result::OK) {
        return -1;
    }

    // Read 'len' bytes
    if (obj->p_spi_->BlockingTransferLL(nullptr, const_cast<uint8_t*>(buf), len) != SpiHandle::Result::OK) {
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

    // Verify object has been properly initialized with a pointer to spi obj
    if (!obj->initialized_ || (obj->p_spi_ == nullptr)) {
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
    if (obj->p_spi_->BlockingTransferLL(&reg, nullptr, 1) != SpiHandle::Result::OK) {
        return -1;
    }

    // Write 'len' bytes
    if (obj->p_spi_->BlockingTransferLL(const_cast<uint8_t*>(buf), nullptr, len)  != SpiHandle::Result::OK) {
        return -1;
    }

    obj->DeselectDevice();

    return 0;
}

} // namespace uvos
