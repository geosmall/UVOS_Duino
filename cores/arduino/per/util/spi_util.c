#include "spi_util.h"

#include "stm32h7xx_ll_spi.h"

/**
 * @brief  return clock freq of an SPI instance
 * @param  spi_inst : SPI instance
 * @retval clock freq of the instance else SystemCoreClock
 */
uint32_t spi_get_clk_freq_inst(SPI_TypeDef *spi_inst) {
  uint32_t spi_freq = SystemCoreClock;
  if (spi_inst != NULL) {
#if defined(SPI1_BASE)
    if (spi_inst == SPI1) {
#if defined(RCC_PERIPHCLK_SPI1) || defined(RCC_PERIPHCLK_SPI123)
#ifdef RCC_PERIPHCLK_SPI1
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
#endif /* RCC_PERIPHCLK_SPI1 */
      if (spi_freq == 0)
#endif /* defined(RCC_PERIPHCLK_SPI1) || defined(RCC_PERIPHCLK_SPI123) */
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif  // SPI1_BASE
#if defined(SPI2_BASE)
    if (spi_inst == SPI2) {
#if defined(RCC_PERIPHCLK_SPI2) || defined(RCC_PERIPHCLK_SPI123) || \
    defined(RCC_PERIPHCLK_SPI23)
#ifdef RCC_PERIPHCLK_SPI2
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
#elif defined(RCC_PERIPHCLK_SPI123)
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI23);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
        spi_freq = HAL_RCC_GetPCLK1Freq();
      }
    }
#endif  // SPI2_BASE
#if defined(SPI3_BASE)
    if (spi_inst == SPI3) {
#if defined(RCC_PERIPHCLK_SPI3) || defined(RCC_PERIPHCLK_SPI123) || \
    defined(RCC_PERIPHCLK_SPI23)
#ifdef RCC_PERIPHCLK_SPI3
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI3);
#elif defined(RCC_PERIPHCLK_SPI123)
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI23);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI_2 and SPI_3. Source CLK is PCKL1 */
        spi_freq = HAL_RCC_GetPCLK1Freq();
      }
    }
#endif  // SPI3_BASE
#if defined(SPI4_BASE)
    if (spi_inst == SPI4) {
#if defined(RCC_PERIPHCLK_SPI4) || defined(RCC_PERIPHCLK_SPI45)
#ifdef RCC_PERIPHCLK_SPI4
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI4);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif  // SPI4_BASE
#if defined(SPI5_BASE)
    if (spi_inst == SPI5) {
#if defined(RCC_PERIPHCLK_SPI5) || defined(RCC_PERIPHCLK_SPI45)
#ifdef RCC_PERIPHCLK_SPI5
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI5);
#else
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI45);
#endif
      if (spi_freq == 0)
#endif
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif  // SPI5_BASE
#if defined(SPI6_BASE)
    if (spi_inst == SPI6) {
#if defined(RCC_PERIPHCLK_SPI6)
      spi_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);
      if (spi_freq == 0)
#endif
      {
        /* SPI1, SPI4, SPI5 and SPI6. Source CLK is PCKL2 */
        spi_freq = HAL_RCC_GetPCLK2Freq();
      }
    }
#endif  // SPI6_BASE
  }
  return spi_freq;
}

/**
 * @brief  Compute H7 delay to use before disabling SPI, calculated
 *         using the SPI instance clock and configured prescaler bits
 *         See https://github.com/stm32duino/Arduino_Core_STM32/issues/1294
 *         Computed delay is a single SPI clock period
 * @param  spi_inst : SPI instance
 * @retval Disable delay in microsecondes
 */
uint32_t spi_compute_disable_delay_us(SPI_TypeDef *spi_inst) {
  // Get the SPI clock frequency from the current hardware configuration.
  // This function returns the SPI input clock frequency in Hertz.
  uint32_t spi_freq = spi_get_clk_freq_inst(spi_inst);

  // Declare variables for the prescaler value and the calculated delay.
  uint32_t disable_delay;
  uint32_t prescaler;

  // Get SPI instance's already configured Baudrate prescaler
  // Returns LL group SPI_LL_EC_BAUDRATEPRESCALER (SPI_CFG1_MBR_2 |
  // SPI_CFG1_MBR_1 | SPI_CFG1_MBR_0)
  uint32_t BaudRatePrescalerBits = LL_SPI_GetBaudRatePrescaler(spi_inst);

  /**
   * The prescaler divides the SPI clock frequency to achieve the desired baud
   * rate. Extract the prescaler value from the BaudRatePrescaler field in the
   * SPI initialization struct.
   * - `handle->Init.BaudRatePrescaler` contains the prescaler encoded in HAL
   * and LL as: SPI_BAUDRATEPRESCALER_2  -> 0x00000000 =
   * LL_SPI_BAUDRATEPRESCALER_DIV2 SPI_BAUDRATEPRESCALER_4  -> 0x10000000 =
   * LL_SPI_BAUDRATEPRESCALER_DIV4 SPI_BAUDRATEPRESCALER_8  -> 0x20000000 =
   * LL_SPI_BAUDRATEPRESCALER_DIV8
   *     ...
   * - To decode it, shift right by `SPI_CFG1_MBR_Pos` (constant offset) and
   * compute `2^(extracted + 1)`. The result is the numeric division factor
   * (e.g., 8 for SPI_BAUDRATEPRESCALER_8).
   */
  prescaler = 1 << ((BaudRatePrescalerBits >> SPI_CFG1_MBR_Pos) + 1);

  /**
   * Compute the disable delay in microseconds.
   * - Multiply the prescaler by 1,000,000 (to convert frequency to
   * microseconds).
   * - Divide by the SPI clock frequency (`spi_freq`) to get the SPI clock
   * period in microseconds.
   * - Add 1 to ensure rounding up (the delay must always be at least 1
   * microsecond). Note: All calculations use integer math, so results are
   * truncated (not rounded).
   */
  disable_delay = ((prescaler * 1000000UL) / spi_freq) + 1;

  // Return the computed delay in microseconds.
  return disable_delay;
}
