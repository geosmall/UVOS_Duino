/**
 ******************************************************************************
 * @file           : w25qxx.h
 * @brief          : Minimal W25Qxx Library Header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 - 2025 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef W25QXX_H_
#define W25QXX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

#ifdef DEBUGxxx
#define W25_DBG(...) printf(__VA_ARGS__);\
                     printf("\n")
#else
#define W25_DBG(...)
#endif

#define W25QXX_VERSION "W25QXX ver. 0.01.00"

#define W25QXX_MANUFACTURER_GIGADEVICE 0xC8
#define W25QXX_MANUFACTURER_WINBOND 0xEF

#define W25QXX_DUMMY_BYTE         0xA5
#define W25QXX_GET_ID             0x9F
#define W25QXX_READ_DATA          0x03
#define W25QXX_WRITE_ENABLE       0x06
#define W25QXX_PAGE_PROGRAM       0x02
#define W25QXX_SECTOR_ERASE	      0x20
#define W25QXX_CHIP_ERASE         0xc7
#define W25QXX_READ_REGISTER_1    0x05

// Hardware abstraction layer function types
#define W25QXX_HAL_MAX_DELAY      0xFFFFFFFFU

typedef enum {
    W25QXX_HAL_RESET = 0U,
    W25QXX_HAL_SET = !W25QXX_HAL_RESET
} w25qxx_flag_status_;

typedef enum {
    W25QXX_HAL_OK = 0,
    W25QXX_HAL_ERROR = 1,
    W25QXX_HAL_TIMEOUT = 2
} w25qxx_hal_status_t;

typedef struct {
    w25qxx_hal_status_t (*spi_transmit)(void *spi_handle, uint8_t *data, uint32_t size, uint32_t timeout);
    w25qxx_hal_status_t (*spi_receive)(void *spi_handle, uint8_t *data, uint32_t size, uint32_t timeout);
    void (*gpio_write)(void *gpio_port, uint16_t gpio_pin, uint8_t state);
    uint32_t (*get_tick)(void);
} w25qxx_hal_t;

typedef struct {
#ifdef W25QXX_QSPI
    void *qspiHandle;
#else
    void *spiHandle;
    void *cs_port;
    uint16_t cs_pin;
#endif
    // Hardware abstraction interface
    w25qxx_hal_t hal;
    
    uint8_t manufacturer_id;
    uint16_t device_id;
    uint32_t block_size;
    uint32_t block_count;
    uint32_t sector_size;
    uint32_t sectors_in_block;
    uint32_t page_size;
    uint32_t pages_in_sector;
} W25QXX_HandleTypeDef;

typedef enum {
    W25QXX_Ok,     // 0
    W25QXX_Err,    // 1
    W25QXX_Timeout // 2
} W25QXX_result_t;

#ifdef W25QXX_QSPI
W25QXX_result_t w25qxx_init(W25QXX_HandleTypeDef *w25qxx, void *qhspi, w25qxx_hal_t *hal_interface);
#else
W25QXX_result_t w25qxx_init(W25QXX_HandleTypeDef *w25qxx, void *hspi, void *cs_port, uint16_t cs_pin, w25qxx_hal_t *hal_interface);
#endif
W25QXX_result_t w25qxx_read(W25QXX_HandleTypeDef *w25qxx, uint32_t address, uint8_t *buf, uint32_t len);
W25QXX_result_t w25qxx_write(W25QXX_HandleTypeDef *w25qxx, uint32_t address, uint8_t *buf, uint32_t len);
W25QXX_result_t w25qxx_erase(W25QXX_HandleTypeDef *w25qxx, uint32_t address, uint32_t len);
W25QXX_result_t w25qxx_chip_erase(W25QXX_HandleTypeDef *w25qxx);

#ifdef __cplusplus
}
#endif

#endif /* W25QXX_H_ */

/*
 * vim: ts=4 et nowrap
 */
