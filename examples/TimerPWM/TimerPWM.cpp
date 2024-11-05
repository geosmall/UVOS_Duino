#include "uvos_brd.h"
#include <array>

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
    timer_cfg.periph = uvos::TimerHandle::Config::Peripheral::TIM_3;
    timer_cfg.period = (1000 - 1); // Set the period

    uvos::TimerHandle::PWMChannelConfig pwm_channels[4];
    // std::array<uvos::TimerHandle::PWMChannelConfig, 3> pwm_channels;

    // For TIM3 Channel 1 on Pin PB4
    pwm_channels[0].channel = TIM_CHANNEL_1;
    pwm_channels[0].pin = Pin(PORTB, 4);
    pwm_channels[0].duty_cycle = 50; // 50% duty cycle
    pwm_channels[0].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[0].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    // For TIM3 Channel 2 on Pin PB5
    pwm_channels[1].channel = TIM_CHANNEL_2;
    pwm_channels[1].pin = Pin(PORTB, 5);
    pwm_channels[1].duty_cycle = 37; // 37% duty cycle
    pwm_channels[1].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[1].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    // For TIM3 Channel 3 on Pin PB0
    pwm_channels[2].channel = TIM_CHANNEL_3;
    pwm_channels[2].pin = Pin(PORTB, 0);
    pwm_channels[2].duty_cycle = 25; // 25% duty cycle
    pwm_channels[2].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[2].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    // For TIM3 Channel 4 on Pin PB1
    pwm_channels[3].channel = TIM_CHANNEL_4;
    pwm_channels[3].pin = Pin(PORTB, 1);
    pwm_channels[3].duty_cycle = 12; // 12% duty cycle
    pwm_channels[3].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[3].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    timer.InitPWM(timer_cfg, pwm_channels, 4);

    // Define the APB1 frequency, target tick, and period
    uint32_t timer_freq_hz = timer.GetFreq(); // Expect 200'000'000 Hz
    uint32_t target_tick_freq_hz = 1'000'000; // 1MHz
    uint32_t period_uSec = 1000; // 1 ms period

    // Calculate prescaler
    uint32_t prescaler = (timer_freq_hz / target_tick_freq_hz) - 1;
    // Calculate auto-reload value, divide by 1'000'000 first to avoid overflow
    uint32_t auto_reload = (target_tick_freq_hz / 1'000'000) * period_uSec - 1;

    timer.SetPrescaler(prescaler);
    timer.SetPeriod(auto_reload);

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
