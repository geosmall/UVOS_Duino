#include "uvos_brd.h"
#include "stm32h7xx_ll_gpio.h"

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

    // Create a test pin object, init as an output
    uvs_gpio test_pin;
    test_pin.pin.port = UVS_GPIOA;
    test_pin.pin.pin  = 5;
    test_pin.mode     = UVS_GPIO_MODE_OUTPUT_PP;
    uvs_gpio_init(&test_pin);
    uvs_gpio_write(&test_pin, 0);

    // Precalculate NsToTicks
    uint32_t ticks = System::NsToTicks(500);

    // Loop forever
    for(;;)
    {
        // LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
        WRITE_REG(GPIOA->ODR, READ_REG(GPIOA->ODR) ^ LL_GPIO_PIN_5);

        // Wait (ticks) DWT ticks
        System::DelayTicks(ticks);
    }
}
