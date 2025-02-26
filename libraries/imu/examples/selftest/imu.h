#ifndef IMU_H
#define IMU_H

#include <cstdint>
#include <cstdio>
#include "uvos.h"

// TDK high-level driver API (includes Icm426xxTransport.h references)
extern "C" {
#include "icm42688p.h"

void inv_spi_chip_select_setup_delay(void);
void inv_spi_chip_select_hold_time(void);
void inv_spi_bus_select_device(void);
void inv_spi_bus_deselect_device(void);
}

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
    /**
     * @brief Construct an IMU object bound to a particular SpiHandle.
     *
     * @param spi Reference to a valid, already-initialized SpiHandle.
     */
    IMU(SpiHandle& spi);

    /**
     * @brief Initialize the IMU hardware and driver.
     * @return 0 on success, negative error code on failure.
     */
    int Init();

    /**
     * @brief Perform a soft reset of the device.
     * @return 0 on success, negative error code on failure.
     */
    int Reset();

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
    int SetAccelODR(ICM426XX_ACCEL_CONFIG0_ODR_t frequency);

    /**
     * @brief Configure the gyroscope Output Data Rate.
     * @param frequency e.g. ICM426XX_GYRO_CONFIG0_ODR_1_KHZ
     * @return 0 on success, negative error code on failure.
     */
    int SetGyroODR(ICM426XX_GYRO_CONFIG0_ODR_t frequency);

    /**
     * @brief Set the accelerometer full-scale range.
     * @param fsr e.g. ICM426XX_ACCEL_CONFIG0_FS_SEL_4g
     * @return 0 on success, negative error code on failure.
     */
    int SetAccelFSR(ICM426XX_ACCEL_CONFIG0_FS_SEL_t fsr);

    /**
     * @brief Set the gyroscope full-scale range.
     * @param fsr e.g. ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps
     * @return 0 on success, negative error code on failure.
     */
    int SetGyroFSR(ICM426XX_GYRO_CONFIG0_FS_SEL_t fsr);

    /**
     * @brief Perform IMU self-test.
     * @param result (ACCEL_SUCCESS<<1 | GYRO_SUCCESS), 3 means both passed.
     * @return 0 on success, negative error code on failure.
     */
	// int RunSelfTest(int* result);
    int RunSelfTest(int* result, int* bias = nullptr);

    /**
     * @brief Read sensor data directly from registers (bypassing FIFO).
     * @return 0 on success, negative error code on failure.
     */
    int ReadDataFromRegisters();

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

private:
    /**
     * @brief TDK driver instance for this IMU.
     * IMPORTANT: The driver struct must have a `struct inv_icm426xx_transport`
     * as its first field.
     */
    struct inv_icm426xx driver_{};

    /**
     * @brief Pointer to the SPI handle your code uses for all SPI transactions.
     */
    SpiHandle& spi_;

    /**
     * @brief IMU chip select pin (using software driven CS).
     */
    GPIO csPin_;

    // uvs_gpio_pin uvs_cs_pin_;
    GPIO_TypeDef* p_cs_port_;
    uint16_t cs_pin_;

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
