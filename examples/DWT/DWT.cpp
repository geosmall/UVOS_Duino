#include "uvos_brd.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_bus.h"

// #define USE_HAL

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
    __HAL_RCC_GPIOA_CLK_ENABLE();

#if defined(USE_HAL)
  GPIO_InitTypeDef  GPIO_InitStruct;
  // Configure IO in output push-pull mode to drive external pin
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#else
    /* Configure IO in output push-pull mode to drive external LED1 */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    /* Reset value is LL_GPIO_PULL_NO */
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);
#endif

#define DELAY_NS 19

    // Precalculate NsToTicks
    uint32_t ticks __attribute__((unused)) = System::NsToTicks(DELAY_NS);

    // Loop forever
    for(;;)
    {

#if defined(USE_HAL)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
#else
        // LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
        WRITE_REG(GPIOA->BSRR, LL_GPIO_PIN_5);
#endif
        // Wait (ticks) DWT ticks
        System::DelayTicks(ticks);
        // delayNanos(DELAY_NS);

#if defined(USE_HAL)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
#else
        // LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
        WRITE_REG(GPIOA->BSRR, LL_GPIO_PIN_5 << 16U);
#endif
        // Wait (ticks) DWT ticks
        System::DelayTicks(ticks);
        // delayNanos(DELAY_NS);

    }
}
