#include "uvos_brd.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_bus.h"

// #define USE_INAV_DELAY
// #define USE_HAL /* Uncomment to use ST HAL gpio vs LL/CMSIS

#define TEST_RCC_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define TEST_GPIO_PORT GPIOA
#define TEST_GPIO_PIN GPIO_PIN_5
#define TEST_LL_GPIO_PIN LL_GPIO_PIN_5

extern "C" void delayNanos(int32_t ns);

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hardware
UVOSboard hardware;

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hardware.Configure();
    hardware.Init();

    /* Enable the GPIO Clock */
    TEST_RCC_GPIO_CLOCK_ENABLE();

#if defined(USE_HAL)
  GPIO_InitTypeDef  GPIO_InitStruct;
  // Configure IO in output push-pull mode to drive external pin
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = TEST_GPIO_PIN;
  HAL_GPIO_Init(TEST_GPIO_PORT, &GPIO_InitStruct);
#else
    /* Configure IO in output push-pull mode to drive external LED1 */
    LL_GPIO_SetPinMode(TEST_GPIO_PORT, TEST_LL_GPIO_PIN, LL_GPIO_MODE_OUTPUT);
    /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
    LL_GPIO_SetPinOutputType(TEST_GPIO_PORT, TEST_LL_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
    LL_GPIO_SetPinSpeed(TEST_GPIO_PORT, TEST_LL_GPIO_PIN, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    /* Reset value is LL_GPIO_PULL_NO */
    LL_GPIO_SetPinPull(TEST_GPIO_PORT, TEST_LL_GPIO_PIN, LL_GPIO_PULL_UP);
#endif

#define DELAY_NS 16

    // Precalculate NsToTicks
    uint32_t ticks __attribute__((unused)) = System::NsToTicks(DELAY_NS);

    // Loop forever
    for(;;)
    {

#if defined(USE_HAL)
        HAL_GPIO_WritePin(TEST_GPIO_PORT, TEST_GPIO_PIN, GPIO_PIN_SET);
#else
        // LL_GPIO_SetOutputPin(TEST_GPIO_PORT, TEST_LL_GPIO_PIN);
        WRITE_REG(TEST_GPIO_PORT->BSRR, TEST_LL_GPIO_PIN);
#endif /* USE_HAL */

        // Wait (ticks) DWT ticks
#if defined(USE_INAV_DELAY)
        delayNanos(DELAY_NS);
#else
        // System::DelayTicks(ticks);
        DELAY_TICKS(12);
#endif /* USE_INAV_DELAY */

#if defined(USE_HAL)
        HAL_GPIO_WritePin(TEST_GPIO_PORT, TEST_GPIO_PIN, GPIO_PIN_RESET);
#else
        // LL_GPIO_ResetOutputPin(TEST_GPIO_PORT, TEST_LL_GPIO_PIN);
        WRITE_REG(TEST_GPIO_PORT->BSRR, TEST_LL_GPIO_PIN << 16U);
#endif /* USE_HAL */

        // Wait (ticks) DWT ticks
#if defined(USE_INAV_DELAY)
        delayNanos(DELAY_NS);
#else
        // System::DelayTicks(ticks);
        DELAY_TICKS(12);
#endif /* USE_INAV_DELAY */

    }
}
