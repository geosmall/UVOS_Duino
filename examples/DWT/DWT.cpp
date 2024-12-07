#include "uvos_brd.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hardware
UVOSboard hardware;

int main(void)
{
    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hardware.Configure();
    hardware.Init();

    // Create a test pin object
    GPIO my_pin;

    // Create a test pin as my_test_pin
    Pin my_test_pin = Pin(PORTA, 5);

    // Initialize it as an OUTPUT
    my_pin.Init(my_test_pin, GPIO::Mode::OUTPUT);

    // Loop forever
    for(;;)
    {
        // Set the onboard LED
        hardware.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Use Toggle to change test pin state
        my_pin.Toggle();

        // Wait 500ms
        System::DelayUs(500'000);
    }
}
