#pragma once

#include "stm32h7xx.h"
#include <stdint.h>
#include <stdbool.h>

///@brief specifies the SPI speed bus in HZ.
#define SPI_SPEED_CLOCK_DEFAULT 4000000

#define SPI_SPEED_CLOCK_DIV2_MHZ ((uint32_t)2)
#define SPI_SPEED_CLOCK_DIV4_MHZ ((uint32_t)4)
#define SPI_SPEED_CLOCK_DIV8_MHZ ((uint32_t)8)
#define SPI_SPEED_CLOCK_DIV16_MHZ ((uint32_t)16)
#define SPI_SPEED_CLOCK_DIV32_MHZ ((uint32_t)32)
#define SPI_SPEED_CLOCK_DIV64_MHZ ((uint32_t)64)
#define SPI_SPEED_CLOCK_DIV128_MHZ ((uint32_t)128)
#define SPI_SPEED_CLOCK_DIV256_MHZ ((uint32_t)256)

// Defines a default timeout delay in milliseconds for the SPI transfer
#define SPI_TRANSFER_TIMEOUT 100

///@brief SPI errors
typedef enum { SPI_OK = 0, SPI_TIMEOUT = 1, SPI_ERROR = 2 } spi_status_e;

// Add C++ guard
#ifdef __cplusplus
extern "C" {
#endif

uint32_t spi_get_clk_freq_inst(SPI_TypeDef *spi_inst);
uint32_t spi_compute_disable_delay_us(SPI_TypeDef *spi_inst);

// Add C++ guard
#ifdef __cplusplus
}
#endif