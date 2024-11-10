#pragma once

#include "per/gpio.h"
#include "per/tim.h"

namespace uvos
{

/** @brief Structure representing a PWM output channel. */
struct PWMOutputChannel
{
    TimerHandle::Config::Peripheral timer_periph; /**< Timer peripheral used. */
    uint32_t channel;                             /**< Timer channel (TIM_CHANNEL_1, etc.). */
    Pin pin;                                      /**< GPIO pin associated with the output. */
    uint32_t pulse_width;                         /**< Current pulse width in microseconds. */
    uint32_t polarity;                            /**< Polarity: TIM_OCPOLARITY_HIGH or TIM_OCPOLARITY_LOW. */
    uint32_t alternate;                           /**< GPIO alternate function. */
};

/** @brief PWMOutput class manages PWM outputs for a specific frequency.
 *
 *  It handles multiple timers and outputs, accommodating the hardware constraints
 *  of the STM32H743 microcontroller.
 */
class PWMOutput
{
  public:
    /** @brief Constructor for PWMOutput.
     *  @param outputs Array of PWMOutputChannel structures defining the outputs.
     *  @param num_outputs Number of outputs in the array.
     *  @param frequency Desired PWM frequency for the outputs.
     */
    PWMOutput(PWMOutputChannel* outputs, size_t num_outputs, uint32_t frequency);

    ~PWMOutput();

    /** @brief Initializes the PWM outputs and timers. */
    void Init();

    /** @brief Sets the pulse width for a specific output.
     *  @param output_index Index of the output to set.
     *  @param pulse_width Pulse width in microseconds.
     */
    void SetPulseWidth(size_t output_index, uint32_t pulse_width);

  private:
    /** @brief Calculates the prescaler and period for a given frequency.
     *  @param frequency Desired PWM frequency.
     *  @param prescaler Calculated prescaler value.
     *  @param period Calculated period value.
     */
    void CalculatePrescalerAndPeriod(uint32_t frequency, uint32_t& prescaler, uint32_t& period);

    // Array of outputs
    PWMOutputChannel* outputs_;
    size_t num_outputs_;

    // PWM frequency for this set of outputs
    uint32_t frequency_;

    // Map of timers used
    struct TimerInfo
    {
        TimerHandle timer_handle;
        TimerHandle::Config::Peripheral periph;
        uint32_t prescaler;
        uint32_t period;
        uint32_t timer_freq;
    };

    // Since dynamic memory allocation is discouraged, we can use a fixed-size array
    static constexpr size_t MAX_TIMERS = 3; // Adjust size as needed
    TimerInfo timers_[MAX_TIMERS];
    size_t num_timers_;
};

} // namespace uvos
