#include "pwm_output.h"
#include "sys/system.h"
#include <cassert>

namespace uvos
{

PWMOutput::PWMOutput(PWMOutputChannel* outputs, size_t num_outputs, uint32_t frequency)
    : outputs_(outputs), num_outputs_(num_outputs), frequency_(frequency), num_timers_(0)
{
}

PWMOutput::~PWMOutput()
{
    // Destructor implementation if needed
}

uint8_t PWMOutput::GetTimerMaxChannels(TimerHandle::Config::Peripheral timer_periph)
{
    switch (timer_periph)
    {
        case TimerHandle::Config::Peripheral::TIM_2:
        case TimerHandle::Config::Peripheral::TIM_3:
        case TimerHandle::Config::Peripheral::TIM_4:
        case TimerHandle::Config::Peripheral::TIM_5:
            return 4; // These timers have 4 channels
        case TimerHandle::Config::Peripheral::TIM_15:
            return 2; // TIM15 has 2 channels
        default:
            // Handle other timers or return 0 as default
            return 0;
    }
}

uint32_t PWMOutput::GetTimerClockFrequency(TimerHandle::Config::Peripheral timer_periph)
{
    uint32_t timer_clk = 0;

    switch (timer_periph)
    {
    case TimerHandle::Config::Peripheral::TIM_15:
        // Timers on APB2
        timer_clk = System::GetPClk2Freq() * 2; // APB2 timers have x2 multiplier if APB prescaler > 1
        break;
    case TimerHandle::Config::Peripheral::TIM_2:
    case TimerHandle::Config::Peripheral::TIM_3:
    case TimerHandle::Config::Peripheral::TIM_4:
    case TimerHandle::Config::Peripheral::TIM_5:
    default:
        // Timers on APB1
        timer_clk = System::GetPClk1Freq() * 2; // APB1 timers have x2 multiplier if APB prescaler > 1
        break;
    }

    return timer_clk;
}

void PWMOutput::CalculatePrescalerAndPeriod(uint32_t frequency,
                                            uint32_t& prescaler,
                                            uint32_t& period,
                                            TimerHandle::Config::Peripheral timer_periph)
{
    uint32_t timer_clk = GetTimerClockFrequency(timer_periph);
    uint32_t target_timer_clk = 1'000'000; // 1 MHz timer clock
    prescaler = (timer_clk / target_timer_clk) - 1;
    period = (target_timer_clk / frequency) - 1;
}

PWMOutput::Result PWMOutput::Init()
{
    // Initialize timers used by outputs
    num_timers_ = 0;

    // First, identify unique timers used
    for (size_t i = 0; i < num_outputs_; ++i)
    {
        PWMOutputChannel& output = outputs_[i];
        bool timer_exists = false;

        // Check if timer is already in timers_ array
        for (size_t j = 0; j < num_timers_; ++j)
        {
            if (timers_[j].periph == output.timer_periph)
            {
                timer_exists = true;
                break;
            }
        }

        // If timer not found, add it to timers_ array and increment counter
        if (!timer_exists)
        {
            // Ensure we don't exceed MAX_TIMERS
            if (num_timers_ >= MAX_TIMERS) return Result::ERR;

            timers_[num_timers_].periph = output.timer_periph;
            num_timers_++;
        }
    }

    // Configure and initialize each timer
    for (size_t i = 0; i < num_timers_; ++i)
    {
        TimerInfo& timer_info = timers_[i];
        TimerHandle::Config timer_cfg;
        timer_cfg.periph = timer_info.periph;

        // Calculate prescaler and period
        CalculatePrescalerAndPeriod(frequency_, timer_cfg.prescaler, timer_cfg.period, timer_info.periph);

        // Store prescaler and period in timer_info
        timer_info.prescaler = timer_cfg.prescaler;
        timer_info.period = timer_cfg.period;

        // Initialize timer
        TimerHandle::Result result = timer_info.timer_handle.Init(timer_cfg);
        if (result != TimerHandle::Result::OK)
            return Result::ERR;

        // Compute timer frequency
        uint32_t timer_clk = GetTimerClockFrequency(timer_info.periph);
        timer_info.timer_freq = timer_clk / (timer_info.prescaler + 1);
    }

    // Initialize PWM channels for each timer
    for (size_t i = 0; i < num_timers_; ++i)
    {
        TimerInfo& timer_info = timers_[i];

        uint8_t max_channels = GetTimerMaxChannels(timer_info.periph);

        TimerHandle::PWMChannelConfig pwm_channels[MAX_PWM_CHANNELS];
        uint8_t num_channels = 0;

        for (size_t j = 0; j < num_outputs_; ++j)
        {
            PWMOutputChannel& output = outputs_[j];

            if (output.timer_periph == timer_info.periph)
            {
                // Ensure we don't exceed the timer's channel capacity
                if (num_channels >= max_channels)
                    return Result::ERR;

                // Ensure the specified output channel is valid
                if (!(((max_channels == 2) && IS_VALID_PWM_2CHANNEL(output.channel)) ||
                      ((max_channels == 4) && IS_VALID_PWM_4CHANNEL(output.channel))))
                    return Result::ERR;

                // Add PWM channel to this timer
                TimerHandle::PWMChannelConfig& pwm_ch = pwm_channels[num_channels++];
                pwm_ch.channel = output.channel;
                pwm_ch.pin = output.pin;
                pwm_ch.polarity = output.polarity;
                pwm_ch.alternate = output.alternate;

                // Convert pulse width from microseconds to ticks
                uint32_t pulse_ticks = static_cast<uint32_t>((static_cast<uint64_t>(output.pulse_width) * timer_info.timer_freq) / 1'000'000ULL);

                // Ensure pulse_ticks does not exceed period
                if (pulse_ticks > timer_info.period) pulse_ticks = timer_info.period;

                pwm_ch.pulse = pulse_ticks;
            }
        }

        // Initialize PWM on this timer
        if (num_channels > 0)
        {
            timer_info.timer_handle.InitPWM(timer_info.timer_handle.GetConfig(), pwm_channels, num_channels);
            // Start PWM
            timer_info.timer_handle.StartPWM();
        }
    }

    return Result::OK;
}

void PWMOutput::SetPulseWidth(size_t output_index, uint32_t pulse_width)
{
    if (output_index >= num_outputs_) return; // Handle error, possibly log or assert

    PWMOutputChannel& output = outputs_[output_index];
    output.pulse_width = pulse_width;

    // Find the timer info associated with this output
    TimerInfo* timer_info = nullptr;
    for (size_t i = 0; i < num_timers_; ++i)
    {
        if (timers_[i].periph == output.timer_periph)
        {
            timer_info = &timers_[i];
            break;
        }
    }

    if (!timer_info) return; // Handle error

    // Convert pulse width in microseconds to timer ticks
    uint32_t pulse_ticks =
      (pulse_width * (timer_info->timer_freq / 1'000'000)); // Assuming pulse_width in microseconds

    // Limit pulse_ticks to period
    if (pulse_ticks > timer_info->period) pulse_ticks = timer_info->period;

    // Set the pulse width using the associated timer
    timer_info->timer_handle.SetPWMPulse(output.channel, pulse_ticks);
}

} // namespace uvos
