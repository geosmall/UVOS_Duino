#ifndef IMU_H
#define IMU_H

#include <cstdint>
#include <cstdio>
#include <array>
#include "uvos.h"

// TDK high-level driver API (includes Icm426xxTransport.h references)
extern "C" {
#include "icm42688p.h"
}

/*
 * ============================================================
 * Accelerometer Sensitivity (LSB per g)
 * ------------------------------------------------------------
 * The scale factor depends on the full-scale range (FSR),
 * which is configured in the ACCEL_CONFIG0 register.
 *
 *  |  Accel Range (g)  | Sensitivity (LSB/g)  |
 *  |-------------------|----------------------|
 *  |       ±16g        |         2048         |
 *  |        ±8g        |         4096         |
 *  |        ±4g        |         8192         |
 *  |        ±2g        |        16384         |
 *
 * ============================================================
 * Gyroscope Sensitivity (LSB per dps)
 * ------------------------------------------------------------
 * The scale factor depends on the full-scale range (FSR),
 * which is configured in the GYRO_CONFIG0 register.
 *
 *  |  Gyro Range (dps) | Sensitivity (LSB/dps) |
 *  |-------------------|-----------------------|
 *  |      ±2000        |          164          |
 *  |      ±1000        |          328          |
 *  |      ±500         |          655          |
 *  |      ±250         |         1311          |
 *
 * ============================================================
 * Note:
 * - The raw sensor output is a signed 16-bit integer.
 * - To convert raw values to physical units:
 *      Accel (g)  = raw_value / sensitivity (LSB/g)
 *      Gyro (dps) = raw_value / sensitivity (LSB/dps)
 * - Make sure the FSR is correctly configured before using
 *   the sensitivity values in calculations.
 * ============================================================
 */

// Use the uvos namespace to prevent having to type uvos:: before all calls
using namespace uvos;

namespace uvos {

/**
 * @brief A C++ wrapper around the TDK ICM-426xx high-level driver, adapted to
 *        use your 'uvos::SpiHandle' and TDK's transport layer definitions.
 */
class IMU
{
public:

    enum PwrState : bool
    {
        POWER_OFF = false,
        POWER_ON = true,
    };

    enum AccelFS : uint8_t
    {
        gpm16 = ICM426XX_ACCEL_CONFIG0_FS_SEL_16g, // (default)
        gpm8 = ICM426XX_ACCEL_CONFIG0_FS_SEL_8g,
        gpm4 = ICM426XX_ACCEL_CONFIG0_FS_SEL_4g,
        gpm2 = ICM426XX_ACCEL_CONFIG0_FS_SEL_2g
    };

    enum GyroFS : uint8_t
    {
        dps2000 = ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps, // (default)
        dps1000 = ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps,
        dps500 = ICM426XX_GYRO_CONFIG0_FS_SEL_500dps,
        dps250 = ICM426XX_GYRO_CONFIG0_FS_SEL_250dps
    };

    enum AccelODR : uint8_t
    {
        accel_odr500 = ICM426XX_ACCEL_CONFIG0_ODR_500_HZ, /*!< 500 Hz (2 ms)*/
        accel_odr1k = ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ, /*!< 1 KHz (1 ms)*/
        accel_odr2k = ICM426XX_ACCEL_CONFIG0_ODR_2_KHZ, /*!< 2 KHz (500 us)*/
        accel_odr4k = ICM426XX_ACCEL_CONFIG0_ODR_4_KHZ, /*!< 4 KHz (250 us)*/
        accel_odr8k = ICM426XX_ACCEL_CONFIG0_ODR_8_KHZ, /*!< 8 KHz (125 us)*/
    };

    enum GyroODR : uint8_t
    {
        gyr_odr500 = ICM426XX_GYRO_CONFIG0_ODR_500_HZ, /*!< 500 Hz (2 ms)*/
        gyr_odr1k = ICM426XX_GYRO_CONFIG0_ODR_1_KHZ, /*!< 1 KHz (1 ms)*/
        gyr_odr2k = ICM426XX_GYRO_CONFIG0_ODR_2_KHZ, /*!< 2 KHz (500 us)*/
        gyr_odr4k = ICM426XX_GYRO_CONFIG0_ODR_4_KHZ, /*!< 4 KHz (250 us)*/
        gyr_odr8k = ICM426XX_GYRO_CONFIG0_ODR_8_KHZ, /*!< 8 KHz (125 us)*/
    };

    enum class Result
    {
        OK,
        ERR,
    };

    /**
     * @brief Construct an IMU object bound to a particular SpiHandle.
     */
    IMU();

    /**
     * @brief Initialize the IMU hardware and driver.
     * @return IMU::OK on success, IMU::ERR on failure.
     */
    Result Init(SpiHandle& spi);

    /**
     * @brief Configure the device full scales and output frequencies.
     * @param acc_fsr_g Accelerometer full-scale range.
     * @param gyr_fsr_dps Gyroscope full-scale range.
     * @param acc_freq Accelerometer Output Data Rate.
     * @param gyr_freq Gyroscope Output Data Rate.
     * @return IMU::OK on success, IMU::ERR on failure.
     */
    Result ConfigureInvDevice(AccelFS acc_fsr_g, GyroFS gyr_fsr_dps, AccelODR acc_freq, GyroODR gyr_freq);

    /**
     * @brief Perform a soft reset of the device.
     * @return 0 on success, negative error code on failure.
     */
    int Reset();

    /**
     * @brief Set power state of device on or off.
     * @return 0 on success, negative error code on failure.
     */
    int SetPwrState(PwrState state);

    /**
     * @brief Enable accelerometer in Low Noise mode.
     * @return 0 on success, negative error code on failure.
     */
    int EnableAccelLNMode();

    /**
     * @brief Disable accelerometer.
     * @return 0 on success, negative error code on failure.
     */
    int DisableAccel();

    /**
     * @brief Enable gyroscope in Low Noise mode.
     * @return 0 on success, negative error code on failure.
     */
    int EnableGyroLNMode();

    /**
     * @brief Disable gyroscope.
     * @return 0 on success, negative error code on failure.
     */
    int DisableGyro();

    /**
     * @brief Configure the accelerometer Output Data Rate.
     * @param frequency e.g. ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ
     * @return 0 on success, negative error code on failure.
     */
    int SetAccelODR(AccelODR frequency);

    /**
     * @brief Configure the gyroscope Output Data Rate.
     * @param frequency e.g. ICM426XX_GYRO_CONFIG0_ODR_1_KHZ
     * @return 0 on success, negative error code on failure.
     */
    int SetGyroODR(GyroODR frequency);

    /**
     * @brief Set the accelerometer full-scale range.
     * @param fsr e.g. ICM426XX_ACCEL_CONFIG0_FS_SEL_4g
     * @return 0 on success, negative error code on failure.
     */
    int SetAccelFSR(AccelFS fsr);

    /**
     * @brief Set the gyroscope full-scale range.
     * @param fsr e.g. ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps
     * @return 0 on success, negative error code on failure.
     */
    int SetGyroFSR(GyroFS fsr);

    /**
     * @brief Get the accelerometer full-scale range.
     * @return Accel sensitivity value (updated upon change to FS value).
     */
    float GetAccelSensitivity() const { return accel_sensitivity_; }

    /**
     * @brief Get the gyroscope full-scale range.
     * @return Gyro sensitivity value (updated upon change to FS value).
     */
    float GetGyroSensitivity()  const { return gyro_sensitivity_; }

    /**
     * @brief Enable the data ready interrupt on INT1 pin.
     * @return 0 on success, negative error code on failure.
     */
    int EnableDataReadyInt1();

    /**
     * @brief Enable the data ready interrupt on INT1 pin.
     * @return 0 on success, negative error code on failure.
     */
    int DisableDataReadyInt1();

    /**
     * @brief Perform IMU self-test.
     * @param result (ACCEL_SUCCESS<<1 | GYRO_SUCCESS), 3 means both passed.
     * @param bias Optional array of 6 int, stores bias values (3 for accel, 3 for gyro).
     * @return 0 on success, negative error code on failure.
     */
    // int RunSelfTest(int* result, int* bias = nullptr);
    int RunSelfTest(int* result, std::array<int, 6>* bias = nullptr);

    /**
     * @brief Read sensor data from registers (bypassing FIFO).
     * @return 0 on success, negative error code on failure.
     */
    int ReadDataFromRegisters();

    /**
     * @brief Read Acc/Gyro data direct from registers (bypassing transport read for speed).
     * @param buf filled with Accel X Y Z followed by Gyro X Y Z
     * @return 0 on success, negative error code on failure.
     */
    // int ReadIMU6(uint8_t *buf);
    int ReadIMU6(std::array<int16_t, 6>& buf);

    /**
     * @brief Read sensor data from FIFO.
     * @return Number of FIFO packets read on success, or negative error code on failure.
     */
    int ReadDataFromFifo();

    /**
     * @brief Provide a user callback for sensor events. This is called by the TDK driver
     *        whenever data is read from registers or FIFO.
     *
     * @param userCb The function pointer for your callback, or nullptr to disable.
     */
    void SetSensorEventCallback(void (*userCb)(inv_icm426xx_sensor_event_t *event));

    // Define maximum read and write sizes for IMU as private static constants
    static constexpr uint32_t IMU_MAX_READ = 255;
    static constexpr uint32_t IMU_MAX_WRITE = 255;
    static constexpr uint32_t NUM_DATA_BYTES = (ACCEL_DATA_SIZE + GYRO_DATA_SIZE);

private:
    /**
     * @brief TDK driver instance for this IMU.
     * IMPORTANT: The driver struct must have a `struct inv_icm426xx_transport`
     * as its first field.
     */
    struct inv_icm426xx driver_{};

    /**
     * @brief Pointer to the SPI handle used for IMU SPI transactions.
     * Initialized to null, to be bound in Init()
     */
    SpiHandle* p_spi_ = nullptr;

    bool initialized_ = false;

    /**
     * @brief IMU chip select pin (using software driven CS).
     */
    GPIO csPin_;

    float accel_sensitivity_{-1.0f};
    float gyro_sensitivity_{-1.0f};

    // CS->CLK delay, MPU6000 - 8ns
    // CS->CLK delay, ICM42688P - 39ns
    static constexpr uint32_t SETUP_TIME_NS  = 39;   // For ICM42688P
    static constexpr uint32_t HOLD_TIME_NS   = 18;   // For ICM42688P

    void SelectDevice();

    void DeselectDevice();

    /**
     * @brief The TDK driver calls this function when new sensor data arrives.
     */
    static void DriverEventCb(inv_icm426xx_sensor_event_t *event);

    /**
     * @brief User-defined callback pointer.
     */
    static void (*userEventCb_)(inv_icm426xx_sensor_event_t *event);

    // -------------------------------------------------------------------------
    // The TDK transport layer requires read_reg, write_reg, configure
    // function pointers with the following signatures:
    //   int foo(struct inv_icm426xx_serif *serif, uint8_t reg, ..., uint32_t len);
    // We'll implement them as static methods. We retrieve the IMU instance via
    //   (IMU*)serif->context.
    // -------------------------------------------------------------------------

    /**
     * @brief  TDK read callback for SPI-based register reads.
     */
    static int spiReadRegs(struct inv_icm426xx_serif *serif,
                           uint8_t                    reg,
                           uint8_t                   *buf,
                           uint32_t                   len);

    /**
     * @brief  TDK write callback for SPI-based register writes.
     */
    static int spiWriteRegs(struct inv_icm426xx_serif *serif,
                            uint8_t                    reg,
                            const uint8_t             *buf,
                            uint32_t                   len);

    /**
     * @brief  TDK configure callback, if used. Often a no-op for many systems.
     */
    static int spiConfigure(struct inv_icm426xx_serif *serif);
};

} // namespace uvos

#endif // IMU_H
