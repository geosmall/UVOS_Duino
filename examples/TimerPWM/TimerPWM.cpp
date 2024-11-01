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

    uvos::TimerHandle timer;
    uvos::TimerHandle::Config timer_cfg;
    timer_cfg.periph = uvos::TimerHandle::Config::Peripheral::TIM_2;
    timer_cfg.period = 1000 - 1; // For a PWM frequency of (timer clock / 1000)

    uvos::TimerHandle::PWMChannelConfig pwm_channels[2];
    pwm_channels[0].channel = TIM_CHANNEL_1;
    pwm_channels[0].pin = Pin(PORTA, 0); // Adjust with your actual pin
    pwm_channels[0].duty_cycle = 50;     // 50% duty cycle
    pwm_channels[0].polarity = TIM_OCPOLARITY_HIGH;

    pwm_channels[1].channel = TIM_CHANNEL_2;
    pwm_channels[1].pin = Pin(PORTA, 1); // Adjust with your actual pin
    pwm_channels[1].duty_cycle = 25;     // 25% duty cycle
    pwm_channels[1].polarity = TIM_OCPOLARITY_HIGH;

    timer.InitPWM(timer_cfg, pwm_channels, 2);
    timer.StartPWM();

    // Loop forever
    for(;;)
    {
        // Set the onboard LED
        hardware.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Wait 500ms
        System::Delay(500);
    }
}
