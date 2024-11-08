#include "per/tim.h"
#include "per/gpio.h"
#include "sys/system.h"
#include "util/hal_map.h"


// To save from digging in reference manual here are some notes:
//
// The following TIMs are on APB1 Clock (RM0433 Rev7 - pg 458):
// - TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM12, TITM13, TIM14
// - LPTIM1
//
// The following TIMs are on APB2 Clock (RM0433 Rev7 - pg 464):
// - TIM1, TIM8, TIM15, TIM16, TIM17,
// - HRTIM
//

namespace uvos
{
/** Private Implementation for TimerHandle */
class TimerHandle::Impl
{
  public:
    TimerHandle::Result        Init(const TimerHandle::Config& config);
    TimerHandle::Result        DeInit();
    const TimerHandle::Config& GetConfig() const { return config_; }

    TimerHandle::Result Start();
    TimerHandle::Result Stop();
    TimerHandle::Result SetPeriod(uint32_t ticks);
    TimerHandle::Result SetPrescaler(uint32_t val);
    uint32_t            GetFreq();
    uint32_t            GetTick();
    uint32_t            GetMs();
    uint32_t            GetUs();

    void DelayTick(uint32_t del);
    void DelayMs(uint32_t del);
    void DelayUs(uint32_t del);

    void SetCallback(TimerHandle::PeriodElapsedCallback cb, void* data)
    {
        if(cb)
        {
            callback_ = cb;
            cb_data_  = data;
        }
    }

    void InternalCallback();

    // PWM-specific methods
    TimerHandle::Result InitPWM(const TimerHandle::Config& config, const TimerHandle::PWMChannelConfig* channels, uint8_t num_channels);
    TimerHandle::Result StartPWM();
    TimerHandle::Result StopPWM();
    TimerHandle::Result SetPWMPulse(uint32_t channel, uint32_t pulse);

    TIM_HandleTypeDef   tim_hal_handle_;

  private:
    TimerHandle::Config config_;

    TimerHandle::PeriodElapsedCallback callback_;
    void*                              cb_data_;

    // PWM-specific members
    TimerHandle::PWMChannelConfig pwm_channels_[4];  // Support up to 4 channels
    uint8_t                       num_pwm_channels_;
};

// Error Handler
static void Error_Handler()
{
#ifdef DEBUG
    asm("bkpt 255");
#endif
    while(1) {}
}

// Global References

static TimerHandle::Impl tim_handles[4];

/** @brief returns a poitner to the private implementation object associated
 *   with the peripheral instance (register base address)
 *  @return Pointer to global tim_handle object or NULL
 */
static TimerHandle::Impl* get_tim_impl_from_instance(TIM_TypeDef* per_instance)
{
    /** Check each impl */
    for(int i = 0; i < 4; i++)
    {
        TimerHandle::Impl* p = &tim_handles[i];
        if(p->tim_hal_handle_.Instance == per_instance)
        {
            return p;
        }
    }
    return NULL;
}

// TIM functions

TimerHandle::Result TimerHandle::Impl::Init(const TimerHandle::Config& config)
{
    const int tim_idx = int(config.periph);
    if(tim_idx >= 4)
        return TimerHandle::Result::ERR;
    config_                             = config;
    constexpr TIM_TypeDef* instances[4] = {TIM2, TIM3, TIM4, TIM5};

    // HAL Initialization
    tim_hal_handle_.Instance = instances[tim_idx];
    tim_hal_handle_.Init.CounterMode
        = config_.dir == TimerHandle::Config ::CounterDir::UP
              ? TIM_COUNTERMODE_UP
              : TIM_COUNTERMODE_DOWN;

    // Defaults to lowest prescale val of 0
    tim_hal_handle_.Init.Prescaler = config_.prescaler;

    // Default to longest period (16-bit timers handled separately for clarity,
    // though 16-bit timers extra bits are probably don't care.
    if(tim_hal_handle_.Instance == TIM2 || tim_hal_handle_.Instance == TIM5)
        tim_hal_handle_.Init.Period = config_.period;
    else
        tim_hal_handle_.Init.Period = (uint16_t)config_.period;

    // Default Clock Division as none.
    tim_hal_handle_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    tim_hal_handle_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    // This function will call MspInit below
    if(HAL_TIM_Base_Init(&tim_hal_handle_) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    sClockSourceConfig.ClockSource             = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(&tim_hal_handle_, &sClockSourceConfig)
       != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&tim_hal_handle_, &sMasterConfig)
       != HAL_OK)
    {
        Error_Handler();
    }

    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerHandle::Impl::DeInit()
{
    if(Stop() != TimerHandle::Result::OK)
    {
        Error_Handler();
    }

    if(HAL_TIM_Base_DeInit(&tim_hal_handle_) != HAL_OK)
    {
        Error_Handler();
    }
    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerHandle::Impl::Start()
{
    if(config_.enable_irq)
    {
        return HAL_TIM_Base_Start_IT(&tim_hal_handle_) == HAL_OK
                   ? TimerHandle::Result::OK
                   : TimerHandle::Result::ERR;
    }
    else
    {
        return HAL_TIM_Base_Start(&tim_hal_handle_) == HAL_OK
                   ? TimerHandle::Result::OK
                   : TimerHandle::Result::ERR;
    }
}

TimerHandle::Result TimerHandle::Impl::Stop()
{
    return HAL_TIM_Base_Stop(&tim_hal_handle_) == HAL_OK
               ? TimerHandle::Result::OK
               : TimerHandle::Result::ERR;
}

TimerHandle::Result TimerHandle::Impl::SetPrescaler(uint32_t val)
{
    config_.prescaler              = val;
    tim_hal_handle_.Instance->PSC  = val;
    tim_hal_handle_.Init.Prescaler = val;
    return Result::OK;
}

TimerHandle::Result TimerHandle::Impl::SetPeriod(uint32_t ticks)
{
    config_.period                = ticks;
    tim_hal_handle_.Instance->ARR = ticks;
    tim_hal_handle_.Init.Period   = ticks;
    return Result::OK;
}

uint32_t TimerHandle::Impl::GetFreq()
{
    // TIM ticks run at 2x PClk
    // there is a switchable 1/2/4 prescalar available
    // that is not yet implemented.
    // Once it is, it should be taken into account here as well.
    uint32_t clkfreq_hz = (System::GetPClk1Freq() * 2);
    uint32_t hz         = clkfreq_hz / (tim_hal_handle_.Instance->PSC + 1);
    return hz;
}

uint32_t TimerHandle::Impl::GetTick()
{
    return tim_hal_handle_.Instance->CNT;
}

uint32_t TimerHandle::Impl::GetMs()
{
    return GetTick() / (GetFreq() / 100000000);
}
uint32_t TimerHandle::Impl::GetUs()
{
    return GetTick() / (GetFreq() / 1000000);
}

void TimerHandle::Impl::DelayTick(uint32_t del)
{
    uint32_t now = GetTick();
    while(GetTick() - now < del) {}
}

void TimerHandle::Impl::DelayMs(uint32_t del)
{
    DelayTick(del * (GetFreq() / 100000000));
}

void TimerHandle::Impl::DelayUs(uint32_t del)
{
    DelayTick(del * (GetFreq() / 1000000));
}

void TimerHandle::Impl::InternalCallback()
{
    if(callback_)
    {
        callback_(cb_data_);
    }
}

TimerHandle::Result TimerHandle::Impl::InitPWM(const TimerHandle::Config& config,
                                               const TimerHandle::PWMChannelConfig* channels,
                                               uint8_t num_channels)
{
    if (num_channels == 0 || num_channels > 4) return TimerHandle::Result::ERR;

    config_ = config;
    num_pwm_channels_ = num_channels;

    // Copy channel configurations
    for (uint8_t i = 0; i < num_pwm_channels_; ++i)
    {
        pwm_channels_[i] = channels[i];
    }

    // Initialize GPIO pins for PWM channels
    for (uint8_t i = 0; i < num_pwm_channels_; ++i)
    {
        GPIO::Config gpio_cfg;
        gpio_cfg.pin = pwm_channels_[i].pin;
        gpio_cfg.mode = GPIO::Mode::AF_PP;
        gpio_cfg.pull = GPIO::Pull::NOPULL;
        gpio_cfg.speed = GPIO::Speed::VERY_HIGH;

        // Use the alternate function from the channel configuration
        gpio_cfg.alternate = pwm_channels_[i].alternate;

        GPIO gpio;
        gpio.Init(gpio_cfg);
    }

    // Configure timer for PWM
    const int tim_idx = int(config_.periph);
    if (tim_idx >= 4) return TimerHandle::Result::ERR;

    constexpr TIM_TypeDef* instances[4] = {TIM2, TIM3, TIM4, TIM5};
    tim_hal_handle_.Instance = instances[tim_idx];

    // Defaults to lowest prescale val of 0
    tim_hal_handle_.Init.Prescaler = config_.prescaler;

    // Default to longest period (16-bit timers handled separately for clarity,
    // though 16-bit timers extra bits are probably don't care.
    if(tim_hal_handle_.Instance == TIM2 || tim_hal_handle_.Instance == TIM5)
        tim_hal_handle_.Init.Period = config_.period;
    else
        tim_hal_handle_.Init.Period = (uint16_t)config_.period;

    tim_hal_handle_.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim_hal_handle_.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim_hal_handle_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&tim_hal_handle_) != HAL_OK)
    {
        Error_Handler();
    }

    // Configure PWM channels
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    for (uint8_t i = 0; i < num_pwm_channels_; ++i)
    {
        sConfigOC.Pulse = pwm_channels_[i].pulse;
        sConfigOC.OCPolarity = pwm_channels_[i].polarity;

        if (HAL_TIM_PWM_ConfigChannel(&tim_hal_handle_, &sConfigOC, pwm_channels_[i].channel) != HAL_OK)
        {
            Error_Handler();
        }
    }

    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerHandle::Impl::StartPWM()
{
    for (uint8_t i = 0; i < num_pwm_channels_; ++i)
    {
        if (HAL_TIM_PWM_Start(&tim_hal_handle_, pwm_channels_[i].channel) != HAL_OK)
        {
            return TimerHandle::Result::ERR;
        }
    }
    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerHandle::Impl::StopPWM()
{
    for (uint8_t i = 0; i < num_pwm_channels_; ++i)
    {
        if (HAL_TIM_PWM_Stop(&tim_hal_handle_, pwm_channels_[i].channel) != HAL_OK)
        {
            return TimerHandle::Result::ERR;
        }
    }
    return TimerHandle::Result::OK;
}

TimerHandle::Result TimerHandle::Impl::SetPWMPulse(uint32_t channel, uint32_t pulse)
{
    // Find the channel index
    for(uint8_t i = 0; i < num_pwm_channels_; ++i)
    {
        if(pwm_channels_[i].channel == channel)
        {
            pwm_channels_[i].pulse = pulse;
            __HAL_TIM_SET_COMPARE(&tim_hal_handle_, channel, pulse);
            return TimerHandle::Result::OK;
        }
    }
    return TimerHandle::Result::ERR; // Channel not found
}

// HAL Functions
extern "C"
{
    void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
    {
        TimerHandle::Impl* impl
            = get_tim_impl_from_instance(tim_baseHandle->Instance);
        /** In the case of TIM6 (DAC usage) there will be no impl.
         *  The preceding enable_irq checks should be false
         *  since the default constructor sets it that way 
         */
        TimerHandle::Config cfg;
        if(impl)
            cfg = impl->GetConfig();
        if(tim_baseHandle->Instance == TIM2)
        {
            __HAL_RCC_TIM2_CLK_ENABLE();
            if(cfg.enable_irq)
            {
                HAL_NVIC_SetPriority(TIM2_IRQn, 0x0f, 0);
                HAL_NVIC_EnableIRQ(TIM2_IRQn);
            }
        }
        else if(tim_baseHandle->Instance == TIM3)
        {
            __HAL_RCC_TIM3_CLK_ENABLE();
            if(cfg.enable_irq)
            {
                HAL_NVIC_SetPriority(TIM3_IRQn, 0x0f, 0);
                HAL_NVIC_EnableIRQ(TIM3_IRQn);
            }
        }
        else if(tim_baseHandle->Instance == TIM4)
        {
            __HAL_RCC_TIM4_CLK_ENABLE();
            if(cfg.enable_irq)
            {
                HAL_NVIC_SetPriority(TIM4_IRQn, 0x0f, 0);
                HAL_NVIC_EnableIRQ(TIM4_IRQn);
            }
        }
        else if(tim_baseHandle->Instance == TIM5)
        {
            __HAL_RCC_TIM5_CLK_ENABLE();
            /** @todo make this conditional based on user config */
            if(cfg.enable_irq)
            {
                HAL_NVIC_SetPriority(TIM5_IRQn, 0x0f, 0);
                HAL_NVIC_EnableIRQ(TIM5_IRQn);
            }
        }
        else if(tim_baseHandle->Instance == TIM6)
        {
            __HAL_RCC_TIM6_CLK_ENABLE();
            /** DAC Peripheral shares IRQ with TIM6
             *  and is implemented as part of DAC
             *  callback structure.
             */
        }
    }

    void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
    {
        if(tim_baseHandle->Instance == TIM2)
        {
            __HAL_RCC_TIM2_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(TIM2_IRQn);
        }
        else if(tim_baseHandle->Instance == TIM3)
        {
            __HAL_RCC_TIM3_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(TIM3_IRQn);
        }
        else if(tim_baseHandle->Instance == TIM4)
        {
            __HAL_RCC_TIM4_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(TIM4_IRQn);
        }
        else if(tim_baseHandle->Instance == TIM5)
        {
            __HAL_RCC_TIM5_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(TIM5_IRQn);
        }
        else if(tim_baseHandle->Instance == TIM6)
        {
            __HAL_RCC_TIM6_CLK_DISABLE();
            /** DAC Peripheral shares IRQ with TIM6
             *  and is implemented as part of DAC
             *  callback structure.
             */
        }
    }

    void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
    {
        if (tim_pwmHandle->Instance == TIM2)
        {
            __HAL_RCC_TIM2_CLK_ENABLE();
            // No interrupts for PWM mode
        }
        else if (tim_pwmHandle->Instance == TIM3)
        {
            __HAL_RCC_TIM3_CLK_ENABLE();
        }
        else if (tim_pwmHandle->Instance == TIM4)
        {
            __HAL_RCC_TIM4_CLK_ENABLE();
        }
        else if (tim_pwmHandle->Instance == TIM5)
        {
            __HAL_RCC_TIM5_CLK_ENABLE();
        }
    }

    void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
    {
        if (tim_pwmHandle->Instance == TIM2)
        {
            __HAL_RCC_TIM2_CLK_DISABLE();
        }
        else if (tim_pwmHandle->Instance == TIM3)
        {
            __HAL_RCC_TIM3_CLK_DISABLE();
        }
        else if (tim_pwmHandle->Instance == TIM4)
        {
            __HAL_RCC_TIM4_CLK_DISABLE();
        }
        else if (tim_pwmHandle->Instance == TIM5)
        {
            __HAL_RCC_TIM5_CLK_DISABLE();
        }
    }
}

// ISRs and event handlers

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    TimerHandle::Impl* impl = get_tim_impl_from_instance(htim->Instance);
    if(impl)
        impl->InternalCallback();
}

extern "C" void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim_handles[(int)TimerHandle::Config::Peripheral::TIM_2]
                            .tim_hal_handle_);
}
extern "C" void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim_handles[(int)TimerHandle::Config::Peripheral::TIM_3]
                            .tim_hal_handle_);
}
extern "C" void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim_handles[(int)TimerHandle::Config::Peripheral::TIM_4]
                            .tim_hal_handle_);
}
extern "C" void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim_handles[(int)TimerHandle::Config::Peripheral::TIM_5]
                            .tim_hal_handle_);
}

// Interface

TimerHandle::Result TimerHandle::Init(const Config& config)
{
    pimpl_ = &tim_handles[int(config.periph)];
    return pimpl_->Init(config);
}

TimerHandle::Result TimerHandle::DeInit()
{
    return pimpl_->DeInit();
}

const TimerHandle::Config& TimerHandle::GetConfig() const
{
    return pimpl_->GetConfig();
}

TimerHandle::Result TimerHandle::Start()
{
    return pimpl_->Start();
}

TimerHandle::Result TimerHandle::Stop()
{
    return pimpl_->Stop();
}

TimerHandle::Result TimerHandle::SetPeriod(uint32_t ticks)
{
    return pimpl_->SetPeriod(ticks);
}

TimerHandle::Result TimerHandle::SetPrescaler(uint32_t val)
{
    return pimpl_->SetPrescaler(val);
}

uint32_t TimerHandle::GetFreq()
{
    return pimpl_->GetFreq();
}

uint32_t TimerHandle::GetTick()
{
    return pimpl_->GetTick();
}

uint32_t TimerHandle::GetMs()
{
    return pimpl_->GetMs();
}

uint32_t TimerHandle::GetUs()
{
    return pimpl_->GetUs();
}

void TimerHandle::DelayTick(uint32_t del)
{
    pimpl_->DelayTick(del);
}

void TimerHandle::DelayMs(uint32_t del)
{
    pimpl_->DelayMs(del);
}

void TimerHandle::DelayUs(uint32_t del)
{
    pimpl_->DelayUs(del);
}

void TimerHandle::SetCallback(PeriodElapsedCallback cb, void* data)
{
    pimpl_->SetCallback(cb, data);
}

TimerHandle::Result TimerHandle::InitPWM(const Config& config, const PWMChannelConfig* channels, uint8_t num_channels)
{
    pimpl_ = &tim_handles[int(config.periph)];
    return pimpl_->InitPWM(config, channels, num_channels);
}

TimerHandle::Result TimerHandle::StartPWM()
{
    return pimpl_->StartPWM();
}

TimerHandle::Result TimerHandle::StopPWM()
{
    return pimpl_->StopPWM();
}

TimerHandle::Result TimerHandle::SetPWMPulse(uint32_t channel, uint32_t pulse)
{
    return pimpl_->SetPWMPulse(channel, pulse);
}

} // namespace uvos


//////////////////////////////////////////////////////////////////
// Delete from here down when everything is retested and works.
//////////////////////////////////////////////////////////////////

#if 0

enum
{
    SCALE_MS,
    SCALE_US,
    SCALE_NS,
    SCALE_LAST,
};

typedef struct
{
    uint32_t          scale[SCALE_LAST];
    TIM_HandleTypeDef htim2;
} uvs_tim;

static void sthal_tim_init();

static uvs_tim tim;

void uvs_tim_init()
{
    tim.scale[SCALE_MS] = 200000;
    tim.scale[SCALE_US] = 200;
    tim.scale[SCALE_NS] = 2;
    sthal_tim_init();
}
void uvs_tim_start()
{
    HAL_TIM_Base_Start(&tim.htim2);
}

uint32_t uvs_tim_get_tick()
{
    return tim.htim2.Instance->CNT;
}

void uvs_tim_delay_tick(uint32_t cnt)
{
    uint32_t now;
    now = uvs_tim_get_tick();
    while(uvs_tim_get_tick() - now < cnt) {}
}
uint32_t uvs_tim_get_ms()
{
    return tim.htim2.Instance->CNT / tim.scale[SCALE_MS];
}
void uvs_tim_delay_ms(uint32_t cnt)
{
    uvs_tim_delay_tick(cnt * tim.scale[SCALE_MS]);
}
uint32_t uvs_tim_get_us()
{
    return tim.htim2.Instance->CNT / tim.scale[SCALE_US];
}

void uvs_tim_delay_us(uint32_t cnt)
{
    uvs_tim_delay_tick(cnt * tim.scale[SCALE_US]);
}

// STM32 HAL Stuff below

static void sthal_tim_init()
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    tim.htim2.Instance           = TIM2;
    tim.htim2.Init.Prescaler     = 0;
    tim.htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    tim.htim2.Init.Period        = 0xffffffff;
    tim.htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    //tim.htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    tim.htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if(HAL_TIM_Base_Init(&tim.htim2) != HAL_OK)
    {
        //    Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(&tim.htim2, &sClockSourceConfig) != HAL_OK)
    {
        //    Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&tim.htim2, &sMasterConfig)
       != HAL_OK)
    {
        //    Error_Handler();
    }
}
#endif
