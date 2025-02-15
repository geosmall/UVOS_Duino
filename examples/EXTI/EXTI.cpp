#include "uvos_brd.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

typedef struct {
    volatile bool led_state;   // Flag toggled when event fires
} BTN_Context_t;

void exti_callback(void* context)
{
    (void)context;
    BTN_Context_t *btn = (BTN_Context_t *)context;  // Cast back to correct type

    // Toggle the LED state from EXTI callback
    btn->led_state = !btn->led_state;
}

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    // Create and initialize the button context
    BTN_Context_t btnCtx = {false};

    GPIO myGpio;
    myGpio.Init(Pin(PORTB, 2), GPIO::Mode::INPUT_IT_RISING, GPIO::Pull::NOPULL);
    myGpio.SetInterruptCallback(exti_callback, &btnCtx);

    // Loop forever
    for(;;)
    {
        // Set the onboard LED based on led_state variable
        hw.SetLed(btnCtx.led_state);

        // Wait a bit
        System::Delay(100);
    }
}
