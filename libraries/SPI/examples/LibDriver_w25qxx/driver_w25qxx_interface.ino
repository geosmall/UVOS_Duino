#include "driver_w25qxx_interface.h"
// #include "delay.h"
// #include "spi.h"
// #include "uart.h"
#include "SPI.h"
#include <stdarg.h>

static SPIClass::Config spi_conf = [] {
    SPIClass::Config cfg{};
    cfg.spi_config.periph = SpiHandle::Config::Peripheral::SPI_3;
    cfg.spi_config.mode = SpiHandle::Config::Mode::MASTER;
    cfg.spi_config.direction = SpiHandle::Config::Direction::TWO_LINES;
    cfg.spi_config.clock_polarity = SpiHandle::Config::ClockPolarity::HIGH;
    cfg.spi_config.clock_phase = SpiHandle::Config::ClockPhase::TWO_EDGE;
#if defined(USE_SOFT_NSS)
    cfg.spi_config.nss = SpiHandle::Config::NSS::SOFT;
#else
    cfg.spi_config.nss = SpiHandle::Config::NSS::HARD_OUTPUT;
#endif
    cfg.spi_config.nss_pulse = SpiHandle::Config::NSSPulseMode::DISABLE;
    cfg.spi_config.pin_config.nss = CS_PIN;
    cfg.spi_config.pin_config.sclk = SCLK_PIN;
    cfg.spi_config.pin_config.miso = MISO_PIN;
    cfg.spi_config.pin_config.mosi = MOSI_PIN;
    cfg.spi_config.baud_prescaler = SpiHandle::Config::BaudPrescaler::PS_64;
    return cfg;
}();
SPIClass SPIbus(spi_conf);   // Handle we'll use to interact with IMU SPI bus

/**
 * @brief Chip select pin (using software driven CS).
 */
GPIO csPin_;

/**
Setup (tCSS) – After you pull /CS low, wait at least 5 ns before driving the first rising clock edge.
Hold (tCHSL) – Keep /CS low for at least 5 ns after the last falling (or rising) clock edge of the byte/word you’re shifting.
Recovery (tCSH) – Once you release /CS, keep it high long enough for the flash to internally finalize the operation:
    ≥ 10 ns if the preceding instruction was a read-type command.
    ≥ 50 ns if the preceding instruction involved a program, erase, or Status-Register write.
*/
static constexpr uint32_t SETUP_TIME_NS    = 10;
static constexpr uint32_t HOLD_TIME_NS     = 10;
static constexpr uint32_t RECOVERY_TIME_NS = 50;

extern "C" {

    uint8_t w25qxx_interface_spi_qspi_init(void);
    uint8_t w25qxx_interface_spi_qspi_deinit(void);
    uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
            uint32_t address, uint8_t address_line, uint8_t address_len,
            uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
            uint8_t dummy, uint8_t* in_buf, uint32_t in_len,
            uint8_t* out_buf, uint32_t out_len, uint8_t data_line);
    void w25qxx_interface_delay_ms(uint32_t ms);
    void w25qxx_interface_delay_us(uint32_t us);
    void w25qxx_interface_debug_print(const char* const fmt, ...);
    void select_device();
    void deselect_device();
    uint8_t spi_write_read(uint8_t* in_buf, uint32_t in_len, uint8_t* out_buf, uint32_t out_len);

    /**
     * @brief  interface spi qspi bus init
     * @return status code
     *         - 0 success
     *         - 1 spi qspi init failed
     * @note   none
     */
    uint8_t w25qxx_interface_spi_qspi_init(void)
    {
        // return spi_init(SPI_MODE_3);
        SPIbus.begin();

        // Configure CS pin for output w/ pullup
        GPIO::Config config;

        config.pin = CS_PIN;
        config.mode = uvos::GPIO::Mode::OUTPUT;
        config.pull = uvos::GPIO::Pull::PULLUP;
        config.speed = uvos::GPIO::Speed::MEDIUM;
        csPin_.Init(config);

        return 0;
    }

    /**
     * @brief  interface spi qspi bus deinit
     * @return status code
     *         - 0 success
     *         - 1 spi qspi deinit failed
     * @note   none
     */
    uint8_t w25qxx_interface_spi_qspi_deinit(void)
    {
        // return spi_deinit();
        return 0;
    }

    /**
     * @brief      interface spi qspi bus write read
     * @param[in]  instruction sent instruction
     * @param[in]  instruction_line instruction phy lines
     * @param[in]  address register address
     * @param[in]  address_line address phy lines
     * @param[in]  address_len address length
     * @param[in]  alternate register address
     * @param[in]  alternate_line alternate phy lines
     * @param[in]  alternate_len alternate length
     * @param[in]  dummy dummy cycle
     * @param[in]  *in_buf pointer to a input buffer
     * @param[in]  in_len input length
     * @param[out] *out_buf pointer to a output buffer
     * @param[in]  out_len output length
     * @param[in]  data_line data phy lines
     * @return     status code
     *             - 0 success
     *             - 1 write read failed
     * @note       none
     */
    uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
            uint32_t address, uint8_t address_line, uint8_t address_len,
            uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
            uint8_t dummy, uint8_t* in_buf, uint32_t in_len,
            uint8_t* out_buf, uint32_t out_len, uint8_t data_line)
    {
        if ((instruction_line != 0) || (address_line != 0) || (alternate_line != 0) || (dummy != 0) || (data_line != 1)) {
            return 1;
        }

        return spi_write_read(in_buf, in_len, out_buf, out_len);
    }

    /**
     * @brief     interface delay ms
     * @param[in] ms time
     * @note      none
     */
    void w25qxx_interface_delay_ms(uint32_t ms)
    {
        System::Delay(ms);
    }

    /**
     * @brief     interface delay us
     * @param[in] us time
     * @note      none
     */
    void w25qxx_interface_delay_us(uint32_t us)
    {
        System::DelayUs(us);
    }

    /**
     * @brief     interface print format data
     * @param[in] fmt format data
     * @note      none
     */
    void w25qxx_interface_debug_print(const char* const fmt, ...)
    {
        char str[256];
        uint16_t len;
        va_list args;

        memset((char*)str, 0, sizeof(char) * 256);
        va_start(args, fmt);
        vsnprintf((char*)str, 255, (char const*)fmt, args);
        va_end(args);

        len = strlen((char*)str);
        // (void)uart_write((uint8_t*)str, len);
        Serial.write((uint8_t*)str, len);
    }

    //------------------------------------------------------------------------------
    // Private SPI helper functions
    //------------------------------------------------------------------------------
    void select_device()
    {
        csPin_.Write(GPIO_PIN_RESET);
        System::DelayNs(SETUP_TIME_NS);
    }

    void deselect_device()
    {
        System::DelayNs(HOLD_TIME_NS);
        csPin_.Write(GPIO_PIN_SET);
        System::DelayNs(RECOVERY_TIME_NS);
    }


    /**
     * @brief      spi bus write read
     * @param[in]  *in_buf pointer to an input buffer
     * @param[in]  in_len input length
     * @param[out] *out_buf pointer to an output buffer
     * @param[in]  out_len output length
     * @return     status code
     *             - 0 success
     *             - 1 write read failed
     * @note       none
     */
    uint8_t spi_write_read(uint8_t* in_buf, uint32_t in_len, uint8_t* out_buf, uint32_t out_len)
    {
        uint8_t res = HAL_OK;

        /* set cs low */
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        select_device();

        /* if in_len > 0 */
        if (in_len > 0) {
            /* transmit the input buffer */
            // res = HAL_SPI_Transmit(&g_spi_handle, in_buf, in_len, 1000);
            SPIbus.transfer(in_buf, nullptr, in_len);

            if (res != HAL_OK) {
                /* set cs high */
                // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
                deselect_device();

                return 1;
            }
        }

        /* if out_len > 0 */
        if (out_len > 0) {
            /* receive to the output buffer */
            // res = HAL_SPI_Receive(&g_spi_handle, out_buf, out_len, 1000);
            SPIbus.transfer(nullptr, out_buf, out_len);
        }

        /* set cs high */
        // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        deselect_device();

        return 0;
    }

} /* extern "C" */