#pragma once

#include "stm32h7xx_hal.h"
#include "uvos_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup utility
    @{
*/


/** global structs, and helper functions for interfacing with the stm32 HAL library
    while it remains a dependancy.
    This file should only be included from source files (c/cpp)
    Including it from a header within libuvos would expose the entire HAL to the users.
    This should be an option for users, but should not be required.
*/

/** \param  *p Pin pin to get
    \return HAL GPIO_TypeDef as used in the HAL from a uvs_gpio_pin input. 
*/
GPIO_TypeDef *uvs_hal_map_get_port(const uvs_gpio_pin *p);

/** \param  *p Pin pin to get
    \return HAL GPIO Pin as used in the HAL from a uvs_gpio_pin input. 
*/
uint16_t uvs_hal_map_get_pin(const uvs_gpio_pin *p);

/** \param  port port clock to enable
*/
void uvs_hal_map_gpio_clk_enable(uvs_gpio_port port);

#ifdef __cplusplus
}
#endif
