#include "pwm_output.h"
#include "sys/system.h"

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

void PWMOutput::CalculatePrescalerAndPeriod(uint32_t frequency, uint32_t& prescaler, uint32_t& period)
{
    uint32_t timer_clk = System::GetPClk1Freq() * 2; // Assuming APB1 timers
    uint32_t target_timer_clk = 1'000'000;           // 1 MHz timer clock for better resolution
    prescaler = (timer_clk / target_timer_clk) - 1;
    period = (target_timer_clk / frequency) - 1;
}

void PWMOutput::Init()
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

        // If timer not found, add it to timers_ array
        if (!timer_exists && num_timers_ < MAX_TIMERS)
        {
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
        CalculatePrescalerAndPeriod(frequency_, timer_cfg.prescaler, timer_cfg.period);

        // Store prescaler and period in timer_info
        timer_info.prescaler = timer_cfg.prescaler;
        timer_info.period = timer_cfg.period;

        // Initialize timer
        timer_info.timer_handle.Init(timer_cfg);

        // Compute timer frequency
        uint32_t timer_clk = System::GetPClk1Freq() * 2; // Assuming APB1 timers
        timer_info.timer_freq = timer_clk / (timer_info.prescaler + 1);
    }

    // Initialize PWM channels for each timer
    for (size_t i = 0; i < num_timers_; ++i)
    {
        TimerInfo& timer_info = timers_[i];

        // Collect PWM channels associated with this timer
        TimerHandle::PWMChannelConfig pwm_channels[4];
        uint8_t num_channels = 0;

        for (size_t j = 0; j < num_outputs_; ++j)
        {
            PWMOutputChannel& output = outputs_[j];

            if (output.timer_periph == timer_info.periph)
            {
                // Add PWM channel to this timer
                TimerHandle::PWMChannelConfig& pwm_ch = pwm_channels[num_channels++];
                pwm_ch.channel = output.channel;
                pwm_ch.pin = output.pin;
                pwm_ch.polarity = output.polarity;
                pwm_ch.alternate = output.alternate;

                // Convert pulse width from microseconds to ticks
                uint32_t pulse_ticks = (output.pulse_width * (timer_info.timer_freq / 1'000'000));
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
