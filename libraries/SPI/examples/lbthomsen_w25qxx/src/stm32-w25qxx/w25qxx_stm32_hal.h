/**
 ******************************************************************************
 * @file           : w25qxx_stm32_hal.h
 * @brief          : STM32 HAL adapter for W25QXX library
 ******************************************************************************
 */

#ifndef W25QXX_STM32_HAL_H_
#define W25QXX_STM32_HAL_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "w25qxx.h"
// #include "stm32f4xx_hal.h"

// STM32 HAL adapter functions
w25qxx_hal_status_t w25qxx_stm32_spi_transmit(void *spi_handle, uint8_t *data, uint32_t size, uint32_t timeout);
w25qxx_hal_status_t w25qxx_stm32_spi_receive(void *spi_handle, uint8_t *data, uint32_t size, uint32_t timeout);
void w25qxx_stm32_gpio_write(void *gpio_port, uint16_t gpio_pin, uint8_t state);
uint32_t w25qxx_stm32_get_tick(void);

// Helper function to get STM32 HAL interface
w25qxx_hal_t w25qxx_get_stm32_hal_interface(void);

#ifdef __cplusplus
}
#endif

#endif /* W25QXX_STM32_HAL_H_ */
