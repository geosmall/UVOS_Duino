#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stm32yyxx.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t dwt_init(void);
void dwt_access(bool ena);

static inline uint32_t dwt_max_sec(void)
{
  return (UINT32_MAX / SystemCoreClock);
};

static inline uint32_t dwt_max_msec(void)
{
  return (UINT32_MAX / (SystemCoreClock / 1000));
};

static inline uint32_t dwt_max_usec(void)
{
  return (UINT32_MAX / (SystemCoreClock / 1000000));
};

static inline uint32_t dwt_getCycles(void)
{
  return (DWT->CYCCNT);
};

#ifdef __cplusplus
}
#endif
