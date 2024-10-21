#pragma once

#include "tim.h" // Include the base TimerHandle class
#include <cstdint>

namespace uvos
{
class TimerPWMHandle
{
  public:
    // Enumeration for PWM channels
    enum class Channel
    {
        CH1 = 0,
        CH2,
        CH3,
        CH4,
    };

    // Configuration for a PWM channel
    struct PWMChannelConfig
    {
        bool         enabled = false;
        uint32_t     pulse   = 0; // Pulse width in timer ticks
        uvs_gpio_pin pin;         // The GPIO pin to use for this channel
        // Additional PWM-specific configurations can be added here
    };

    // Constructor
    TimerPWMHandle();

    // Initialize the PWM handle with a TimerHandle
    TimerHandle::Result Init(TimerHandle& timer);

    // Configure a PWM channel
    TimerHandle::Result ConfigPWMChannel(Channel ch, const PWMChannelConfig& config);

    // Set duty cycle for a PWM channel
    TimerHandle::Result SetPWMDutyCycle(Channel ch, uint32_t pulse);

  private:
    // Pointer to the base timer handle
    TIM_HandleTypeDef* tim_handle_;

    // Store PWM channel configurations
    PWMChannelConfig pwm_channel_configs_[4];

    // Configure hardware for PWM channel
    TimerHandle::Result ConfigurePWMChannelHardware(Channel ch);

    // Configure GPIO for PWM channel
    TimerHandle::Result ConfigureGPIOForChannel(Channel ch);

    // Utility functions
    static TimerHandle::Result checkPinMatchTimPwm(GPIO_InitTypeDef* init, uvs_gpio_pin pin, int p_num);

};

} // namespace uvos
