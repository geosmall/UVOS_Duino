#include "uvos_brd.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

int main(void)
{
    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    //Initialize serial and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    // prints title with ending line break
    Serial.println("Arduino HardwareSerial test...");

    // HardwareSerial::HardwareSerial(PinName _rx, PinName _tx, PinName _rts, PinName _cts)
    HardwareSerial Serial2(PA_3, PA_2);

    //Initialize serial and wait for port to open:
    Serial2.begin(115200);
    while (!Serial2) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    uint8_t i;

    // Loop forever
    for(;;)
    {
        Serial.println(i++);

        // Set the onboard LED
        hw.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Wait 500ms
        System::Delay(500);
    }
}
