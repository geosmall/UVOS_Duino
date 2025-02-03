#include "uvos_brd.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

volatile bool led_state = false;


void exti_callback(void)
{
    // Toggle the LED state from EXTI callback
    led_state = !led_state;

}

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    GPIO myGpio;
    myGpio.Init(Pin(PORTB, 2), GPIO::Mode::INPUT_IT_RISING, GPIO::Pull::NOPULL);
    myGpio.SetInterruptCallback(exti_callback);

    // Loop forever
    for(;;)
    {
        // Set the onboard LED based on led_state variable
        hw.SetLed(led_state);

        // Wait a bit
        System::Delay(10);
    }
}
