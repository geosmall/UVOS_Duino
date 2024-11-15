#pragma once

#include "per/gpio.h"
#include "per/tim.h"

namespace uvos
{

/** @brief Enum class representing timer channels. */
enum class TimChannel : uint32_t
{
    CH_1 = TIM_CHANNEL_1,
    CH_2 = TIM_CHANNEL_2,
    CH_3 = TIM_CHANNEL_3,
    CH_4 = TIM_CHANNEL_4,
};

/** @brief Enum class representing output polarity. */
enum class TimPolarity : uint32_t
{
    HIGH = TIM_OCPOLARITY_HIGH,
    LOW = TIM_OCPOLARITY_LOW,
};

/** @brief Structure representing a PWM output channel. */
struct PWMOutputChannel
{
    TimerHandle::Config::Peripheral timer_periph; /**< Timer peripheral used. */
    TimChannel channel;                           /**< Timer channel (CHANNEL_1, etc.). */
    Pin pin;                                      /**< GPIO pin associated with the output. */
    uint32_t pulse_width;                         /**< Current pulse width in microseconds. */
    TimPolarity polarity;                         /**< Polarity: HIGH or LOW. */
    uint32_t alternate;                           /**< GPIO alternate function. */
};

constexpr int MAX_PWM_CHANNELS = 4;
constexpr int PWM_TIMER_FREQ = 1'000'000;
constexpr int PWM_MIN_FREQ = 20;
constexpr int PWM_MAX_FREQ = 12000;

/** @brief PWMOutput class manages PWM outputs for a specific frequency.
 *
 *  It handles multiple timers and outputs, accommodating the hardware constraints
 *  of the STM32H743 microcontroller.
 */
class PWMOutput
{
  public:
    /** @brief Result enum for PWMOutput operations. */
    enum class Result
    {
        OK,          /**< Operation successful. */
        ERR,         /**< General error. */
    };

    /** @brief Constructor for PWMOutput.
     *  @param outputs Array of PWMOutputChannel structures defining the outputs.
     *  @param num_outputs Number of outputs in the array.
     *  @param frequency Desired PWM frequency for the outputs.
     */
    PWMOutput(PWMOutputChannel* outputs, size_t num_outputs, uint32_t frequency);

    ~PWMOutput();

    /** @brief Initializes the PWM outputs and timers.
     *  @return ERR if an error occurred, OK otherwise.
     */
    Result Init();

    /** @brief Sets the pulse width for a specific output.
     *  @param output_index Index of the output to set.
     *  @param pulse_width Pulse width in microseconds.
     */
    void SetPulseWidth(size_t output_index, uint32_t pulse_width);

  private:
    /** @brief Gets the maximum number of channels for a given timer.
     *  @param timer_periph Timer peripheral.
     *  @return Maximum number of channels supported by the timer.
     */
    uint8_t GetTimerMaxChannels(TimerHandle::Config::Peripheral timer_periph);

    /** @brief Gets a numerical index for a given TimChannel.
     *  @param channel TimChannel to get index for.
     *  @return Numerical index for the channel, 0 if invalid.
     */
    uint8_t GetChannelIndex(TimChannel channel);

    /** @brief Gets the timer clock frequency based on the timer peripheral.
     *  @param timer_periph Timer peripheral to get clock frequency for.
     *  @return Timer clock frequency in Hz.
     */
    uint32_t GetTimerClockFrequency(TimerHandle::Config::Peripheral timer_periph);

    /** @brief Calculates the prescaler and period for a given frequency.
     *  @param frequency Desired PWM frequency.
     *  @param prescaler Calculated prescaler value.
     *  @param period Calculated period value.
     *  @param timer_periph Timer peripheral to calculate for.
     *  @return ERR if an error occurred, OK otherwise.
     */
    PWMOutput::Result CalculatePrescalerAndPeriod(uint32_t frequency,
                                                  uint32_t* prescaler,
                                                  uint32_t* period,
                                                  TimerHandle::Config::Peripheral timer_periph);

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
    static constexpr size_t MAX_TIMERS = 4; // Max number of timers in a PWMOutput object

    TimerInfo timers_[MAX_TIMERS];
    size_t num_timers_;
};

} // namespace uvos
