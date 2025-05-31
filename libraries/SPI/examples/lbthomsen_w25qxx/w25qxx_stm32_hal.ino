/**
 ******************************************************************************
 * @file           : w25qxx_stm32_hal.c
 * @brief          : STM32 HAL adapter for W25QXX library
 ******************************************************************************
 */

#include "src/stm32-w25qxx/w25qxx_stm32_hal.h"
// #include "stm32h7xx_hal.h"

extern "C" {

    w25qxx_hal_status_t w25qxx_stm32_spi_transmit(void* spi_handle, uint8_t* data, uint32_t size, uint32_t timeout);
    w25qxx_hal_status_t w25qxx_stm32_spi_receive(void* spi_handle, uint8_t* data, uint32_t size, uint32_t timeout);
    void w25qxx_stm32_gpio_write(void* gpio_port, uint16_t gpio_pin, uint8_t state);
    uint32_t w25qxx_stm32_get_tick(void);
    w25qxx_hal_t w25qxx_get_stm32_hal_interface(void);

    w25qxx_hal_status_t w25qxx_stm32_spi_transmit(void* spi_handle, uint8_t* data, uint32_t size, uint32_t timeout)
    {
        HAL_StatusTypeDef status = HAL_SPI_Transmit((SPI_HandleTypeDef*)spi_handle, data, size, timeout);
        return (status == HAL_OK) ? W25QXX_HAL_OK : W25QXX_HAL_ERROR;
    }

    w25qxx_hal_status_t w25qxx_stm32_spi_receive(void* spi_handle, uint8_t* data, uint32_t size, uint32_t timeout)
    {
        HAL_StatusTypeDef status = HAL_SPI_Receive((SPI_HandleTypeDef*)spi_handle, data, size, timeout);
        return (status == HAL_OK) ? W25QXX_HAL_OK : W25QXX_HAL_ERROR;
    }

    void w25qxx_stm32_gpio_write(void* gpio_port, uint16_t gpio_pin, uint8_t state)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef*)gpio_port, gpio_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    uint32_t w25qxx_stm32_get_tick(void)
    {
        return HAL_GetTick();
    }

    w25qxx_hal_t w25qxx_get_stm32_hal_interface(void)
    {
        w25qxx_hal_t interface = {
            .spi_transmit = w25qxx_stm32_spi_transmit,
            .spi_receive = w25qxx_stm32_spi_receive,
            .gpio_write = w25qxx_stm32_gpio_write,
            .get_tick = w25qxx_stm32_get_tick
        };
        return interface;
    }

} /* extern "C" */