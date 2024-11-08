#include "uvos_brd.h"
#include <array>

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hardware
UVOSboard hardware;

// Helper function to convert a duty cycle to a pulse width
static inline uint32_t duty_cycle_to_pulse(uint32_t period, uint32_t duty_cycle)
{
    return ((period * duty_cycle) / 100);
}

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

    uint32_t timer_freq_hz = (System::GetPClk1Freq() * 2);
    uint32_t target_tick_freq_hz = 1'000'000; // 1 MHz
    uint32_t period_uSec = 1000; // 1 ms period

    // Calculate prescaler
    uint32_t prescale_val = (timer_freq_hz / target_tick_freq_hz) - 1;

    uvos::TimerHandle timer;
    uvos::TimerHandle::Config timer_cfg;
    timer_cfg.periph = uvos::TimerHandle::Config::Peripheral::TIM_3;
    timer_cfg.prescaler = prescale_val; // Set the prescaler 1 MHz
    timer_cfg.period = (period_uSec - 1); // Set the period to 1 ms

    uvos::TimerHandle::PWMChannelConfig pwm_channels[4];
    // std::array<uvos::TimerHandle::PWMChannelConfig, 3> pwm_channels;

    // For TIM3 Channel 1 on Pin PB4
    pwm_channels[0].channel = TIM_CHANNEL_1;
    pwm_channels[0].pin = Pin(PORTB, 4);
    pwm_channels[0].pulse = duty_cycle_to_pulse(timer_cfg.period, 50); // 50% duty cycle
    pwm_channels[0].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[0].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    // For TIM3 Channel 2 on Pin PB5
    pwm_channels[1].channel = TIM_CHANNEL_2;
    pwm_channels[1].pin = Pin(PORTB, 5);
    pwm_channels[1].pulse = duty_cycle_to_pulse(timer_cfg.period, 37); // 37% duty cycle
    pwm_channels[1].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[1].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    // For TIM3 Channel 3 on Pin PB0
    pwm_channels[2].channel = TIM_CHANNEL_3;
    pwm_channels[2].pin = Pin(PORTB, 0);
    pwm_channels[2].pulse = duty_cycle_to_pulse(timer_cfg.period, 25);; // 25% duty cycle
    pwm_channels[2].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[2].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    // For TIM3 Channel 4 on Pin PB1
    pwm_channels[3].channel = TIM_CHANNEL_4;
    pwm_channels[3].pin = Pin(PORTB, 1);
    pwm_channels[3].pulse = duty_cycle_to_pulse(timer_cfg.period, 12);; // 12% duty cycle
    pwm_channels[3].polarity = TIM_OCPOLARITY_HIGH;
    pwm_channels[3].alternate = GPIO_AF2_TIM3; // Specify the correct alternate function

    timer.InitPWM(timer_cfg, pwm_channels, 4);

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
