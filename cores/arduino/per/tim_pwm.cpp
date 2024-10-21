#include "tim_pwm.h"
#include "stm32h7xx_hal.h"
#include "gpio.h" // Include the GPIO utility class
#include "util/hal_map.h" // For uvs_hal_map_get_port and uvs_hal_map_get_pin
extern "C"
{
#include "per/util/spi_util.h" // For uvs_pin_cmp
}

namespace uvos
{

TimerPWMHandle::TimerPWMHandle()
    : tim_handle_(nullptr)
{
}

TimerHandle::Result TimerPWMHandle::Init(TimerHandle& timer)
{
    // Get the TIM handle from the TimerHandle
    tim_handle_ = timer.GetTIMHandle();

    if (!tim_handle_)
        return TimerHandle::Result::ERR;

    // Initialize PWM using HAL functions
    if (HAL_TIM_PWM_Init(tim_handle_) != HAL_OK)
    {
        return TimerHandle::Result::ERR;
    }

    // Configure each enabled channel
    for (int i = 0; i < 4; ++i)
    {
        if (pwm_channel_configs_[i].enabled)
        {
            Channel ch                  = static_cast<Channel>(i);
            TimerHandle::Result res = ConfigurePWMChannelHardware(ch);
            if (res != TimerHandle::Result::OK)
                return res;
        }
    }

    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerPWMHandle::ConfigPWMChannel(Channel ch, const PWMChannelConfig& config)
{
    // Update the configuration
    pwm_channel_configs_[static_cast<int>(ch)] = config;

    // If PWM is already initialized, reconfigure the hardware
    if (tim_handle_ && tim_handle_->State == HAL_TIM_STATE_BUSY)
    {
        return ConfigurePWMChannelHardware(ch);
    }

    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerPWMHandle::SetPWMDutyCycle(Channel ch, uint32_t pulse)
{
    // Update the pulse width in the configuration
    pwm_channel_configs_[static_cast<int>(ch)].pulse = pulse;

    // Update the CCR register directly
    switch (ch)
    {
        case Channel::CH1:
            tim_handle_->Instance->CCR1 = pulse;
            break;
        case Channel::CH2:
            tim_handle_->Instance->CCR2 = pulse;
            break;
        case Channel::CH3:
            tim_handle_->Instance->CCR3 = pulse;
            break;
        case Channel::CH4:
            tim_handle_->Instance->CCR4 = pulse;
            break;
    }

    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerPWMHandle::ConfigurePWMChannelHardware(Channel ch)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Retrieve the configuration for the channel
    const PWMChannelConfig& config = pwm_channel_configs_[static_cast<int>(ch)];

    // Configure the GPIO pin associated with the channel
    TimerHandle::Result gpio_res = ConfigureGPIOForChannel(ch);
    if (gpio_res != TimerHandle::Result::OK)
    {
        return gpio_res;
    }

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = config.pulse;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

    uint32_t hal_channel = 0;
    switch (ch)
    {
        case Channel::CH1:
            hal_channel = TIM_CHANNEL_1;
            break;
        case Channel::CH2:
            hal_channel = TIM_CHANNEL_2;
            break;
        case Channel::CH3:
            hal_channel = TIM_CHANNEL_3;
            break;
        case Channel::CH4:
            hal_channel = TIM_CHANNEL_4;
            break;
    }

    if (HAL_TIM_PWM_ConfigChannel(tim_handle_, &sConfigOC, hal_channel) != HAL_OK)
    {
        return TimerHandle::Result::ERR;
    }

    if (HAL_TIM_PWM_Start(tim_handle_, hal_channel) != HAL_OK)
    {
        return TimerHandle::Result::ERR;
    }

    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerPWMHandle::ConfigureGPIOForChannel(Channel ch)
{
    // Retrieve the configuration for the channel
    const PWMChannelConfig& config = pwm_channel_configs_[static_cast<int>(ch)];

    // Get the GPIO pin from the configuration
    uvs_gpio_pin pin = config.pin;

    // Create GPIO_InitTypeDef
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

    // Determine the timer and channel index for pins_periphs_tim_pwm
    int p_num = 0; // Adjust this based on the timer and channel
    // For TIM2, ch1 to ch4 are indices 0 to 3

    // For simplicity, let's assume we only support TIM2 for now
    p_num = static_cast<int>(ch); // ch1=0, ch2=1, ch3=2, ch4=3

    // Check if the pin matches
    if (checkPinMatchTimPwm(&GPIO_InitStruct, pin, p_num) != TimerHandle::Result::OK)
    {
        return TimerHandle::Result::ERR;
    }

    // Set the pin number
    GPIO_InitStruct.Pin = (1 << pin.pin);

    // Enable the GPIO clock
    uvs_hal_map_gpio_clk_enable(pin.port);

    // Get the GPIO port
    GPIO_TypeDef* GPIO_Port = uvs_hal_map_get_port(&pin);

    // Initialize the GPIO pin
    HAL_GPIO_Init(GPIO_Port, &GPIO_InitStruct);

    return TimerHandle::Result::OK;
}

// Define the pin and alternate function mapping for PWM channels
typedef struct
{
    uvs_gpio_pin pin;
    uint8_t      alt;
} pin_alt_tim_pwm;

// Define a marker for the end of the pin array
static pin_alt_tim_pwm pins_none_tim_pwm = {{UVS_GPIOX, 0}, 255};

// TIM2 Channel 1
static pin_alt_tim_pwm tim2_pins_ch1[] = {
    {{UVS_GPIOA, 0}, GPIO_AF1_TIM2},
    {{UVS_GPIOA, 5}, GPIO_AF1_TIM2},
    {{UVS_GPIOA, 15}, GPIO_AF1_TIM2},
    pins_none_tim_pwm // End marker
};

// TIM2 Channel 2
static pin_alt_tim_pwm tim2_pins_ch2[] = {
    {{UVS_GPIOA, 1}, GPIO_AF1_TIM2},
    {{UVS_GPIOB, 3}, GPIO_AF1_TIM2},
    pins_none_tim_pwm // End marker
};

// TIM2 Channel 3
static pin_alt_tim_pwm tim2_pins_ch3[] = {
    {{UVS_GPIOA, 2}, GPIO_AF1_TIM2},
    {{UVS_GPIOB, 10}, GPIO_AF1_TIM2},
    pins_none_tim_pwm // End marker
};

// TIM2 Channel 4
static pin_alt_tim_pwm tim2_pins_ch4[] = {
    {{UVS_GPIOA, 3}, GPIO_AF1_TIM2},
    {{UVS_GPIOB, 11}, GPIO_AF1_TIM2},
    pins_none_tim_pwm // End marker
};

// Arrays of pointers to pin arrays per peripheral
static pin_alt_tim_pwm* pins_periphs_tim_pwm[] = {
    tim2_pins_ch1, tim2_pins_ch2, tim2_pins_ch3, tim2_pins_ch4,
    // For other timers, adjust the indexing accordingly
};

// Implement checkPinMatchTimPwm function
TimerHandle::Result
TimerPWMHandle::checkPinMatchTimPwm(GPIO_InitTypeDef* init, uvs_gpio_pin pin, int p_num)
{
    for(int i = 0; i < 3; i++)
    {
        if(uvs_pin_cmp(&pins_periphs_tim_pwm[p_num][i].pin, &pins_none_tim_pwm.pin))
        {
            /* skip */
        }
        else if(uvs_pin_cmp(&pins_periphs_tim_pwm[p_num][i].pin, &pin))
        {
            init->Alternate = pins_periphs_tim_pwm[p_num][i].alt;
            return TimerHandle::Result::OK;
        }
    }
    return TimerHandle::Result::ERR;
}

} // namespace uvos
