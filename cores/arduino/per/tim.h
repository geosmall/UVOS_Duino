#pragma once
#ifndef UVS_TIM_H
#define UVS_TIM_H

#include <cstdint>
#include "util/hal_map.h"

namespace uvos
{
/** @brief Hardare timer peripheral support.
 ** 
 ** Supports general-function TIM peripherals:
 ** - TIM2, TIM3, TIM4, TIM5, TIM15
 **
 ** UVOSboard utilizes TIM2 for timing/delay purposes. It is configured to run
 ** at the maximum frequency: typically 200MHz or 240MHz (boost) 
 ** for measuring/delaying for very short periods.
 **
 ** The GetUs/GetMs functions are available for convenience (and backwards compatibility),
 ** but to avoid wrapping errors on math when doing time-delta calculations, using ticks is 
 ** recommended. The data can be converted to the final time-base after getting the difference
 ** in ticks. (Using GetFreq() can be used for these time-base calculations).
 **
 ** User callbacks can be set, and changed at any point during operation. However,
 ** the Config::enable_irq must be set to true when initializing for the interrupts
 ** to turn on and function.
 **
 ** @todo Fix issues with realtime getters, and wrapping of the timer(s).
 **     - This very noticeable with default settings for the 16-bit counters.
 ** @todo Other General purpose timers
 ** @todo Non-internal clock sources
 ** @todo Use of the four-tim channels per tim
 **     - PWM, etc.
 **     - InputCapture/OutputCompare, etc.
 ** @todo HRTIM
 ** @todo Advanced timers (TIM1/TIM8)
 ** */
class TimerHandle
{
  public:
    /** @brief Configuration struct for the Peripheral
     *  @note These settings are used during initialization
     *   and changing them afterwards may not have the desired effect.
     */
    struct Config
    {
        /** @brief Hardware Timer to configure, and use. */
        enum class Peripheral
        {
            TIM_2 = 0, /**< 32-bit counter, 4 CC channels, APB1 */
            TIM_3,     /**< 16-bit counter, 4 CC channels, APB1 */
            TIM_4,     /**< 16-bit counter, 4 CC channels, APB1 */
            TIM_5,     /**< 32-bit counter, 4 CC channels, APB1 */
            TIM_15,    /**< 16-bit counter, 2 CC channels, APB2 */
        };

        /** @brief Direction of the auto-reload counter. 
         ** @todo Add support for the various  
         ** versions of Up/Down counters.
         ** */
        enum class CounterDir
        {
            UP = 0,
            DOWN,
        };

        Peripheral periph; /**< Hardware Peripheral */
        CounterDir dir;    /**< Counter direction */

        /** @brief prescaler value for TIM peripheral.  Must be 0 min to 0xffff max.
         * @note TIM2-5 are all on APB1, so the prescaler is based on the APB1 clock.
         */
        uint32_t prescaler;

        /** @brief period in ticks at TIM frequency that counter will reset based on dir
         *  @note TIM3 and TIM4 are both 16-bit timers. So the period maximum is 0xffff.
        */
        uint32_t period;

        bool     enable_irq; /**< Enable interrupt for user based callback */

        /* @brief Constructor for default states */
        Config()
        : periph(Peripheral::TIM_2),
          dir(CounterDir::UP),
          prescaler(0),
          period(0xffffffff),
          enable_irq(false)
        {
        }
    };

    /** @brief PWM Output Channel Configuration */
    struct PWMChannelConfig
    {
        uint32_t channel;    /**< Channel number: TIM_CHANNEL_1, TIM_CHANNEL_2, etc. */
        Pin pin;             /**< GPIO Pin associated with the channel */
        uint32_t pulse;      /**< Pulse value to be loaded into CCRx (0x0000 to 0xFFFF) */
        uint32_t polarity;   /**< Polarity: TIM_OCPOLARITY_HIGH or TIM_OCPOLARITY_LOW */
        uint32_t alternate;  /**< Alternate function number for the GPIO pin */

        // Default constructor
        PWMChannelConfig()
            : channel(0), pin(Pin()), pulse(0), polarity(0), alternate(0) {}

        // Constructor with parameters, pulse defaults to 0
        PWMChannelConfig(uint32_t ch, Pin p, uint32_t pol, uint32_t alt)
            : channel(ch), pin(p), pulse(0), polarity(pol), alternate(alt) {}

    };

    /** @brief Return values for TIM funcitons. */
    enum class Result
    {
        OK,
        ERR,
    };

    /** @brief User Callback type that will fire at the end of each timer 
     *   period. This requires that Config::enable_irq is true before Init
     *  @param data pointer to arbitrary user-provided data
    */
    typedef void (*PeriodElapsedCallback)(void* data);

    TimerHandle() : pimpl_(nullptr) {}
    TimerHandle(const TimerHandle& other) = default;
    TimerHandle& operator=(const TimerHandle& other) = default;
    ~TimerHandle() {}

    /** @brief Initializes the timer according to the configuration */
    Result Init(const Config& config);

    /** @brief Deinitializes the timer */
    Result DeInit();

    /** @brief Returns a const reference to the Config struct */
    const Config& GetConfig() const;

    /** @brief Sets the Prescalar applied to the TIM peripheral. 
     * 
     ** This can be any number up to 0xffff 
     ** This will adjust the rate of ticks:
     ** Calculated as APBN_Freq / prescalar per tick
     ** where APBN is APB1 for Most general purpose timers,
     ** and APB2 for HRTIM,a nd the advanced timers. 
     ** This can be changed "on-the-fly" 
     ** */
    Result SetPrescaler(uint32_t val);

    /** @brief Sets the period of the Timer.
     * 
     ** This is the number of ticks it takes before it wraps back around.
     ** For self-managed timing, this can be left at the default. (0xffff for 16-bit
     ** and 0xffffffff for 32-bit timers). 
     ** This can be changed "on-the-fly" 
     ** */
    Result SetPeriod(uint32_t ticks);

    /** @brief Starts the TIM peripheral specified by Config */
    Result Start();

    /** @brief Stops the TIM peripheral specified by Config */
    Result Stop();

    /** @brief Returns the frequency of each tick of the timer in Hz */
    uint32_t GetFreq();

    /** @brief Returns the number of counter position. 
     * 
     ** This increments according to Config::CounterDir, 
     ** and wraps around at the specified period (maxing out 
     ** at 2^16 or 2^32 depending on the chosen TIM peripheral. */
    uint32_t GetTick();

    /** @brief Returns the ticks scaled as milliseconds 
     **
     ** Use care when using for measurements and ensure that 
     ** the TIM period can handle the maximum desired measurement.
     ***/
    uint32_t GetMs();

    /** @brief Returns the ticks scaled as microseconds 
     **
     ** Use care when using for measurements and ensure that 
     ** the TIM period can handle the maximum desired measurement.
     ***/
    uint32_t GetUs();

    /** @brief Stay within this function for del ticks */
    void DelayTick(uint32_t del);

    /** @brief Stay within this function for del milliseconds */
    void DelayMs(uint32_t del);

    /** @brief Stay within this function for del microseconds */
    void DelayUs(uint32_t del);

    /** @brief Sets the PeriodElapsedCallback that will fire 
     *   whenever the timer reaches the end of it's period.
     *  @param cb user callback
     *  @param data optional pointer to arbitrary data (defaults to nullptr)
    */
    void SetCallback(PeriodElapsedCallback cb, void* data = nullptr);

    /** @brief Initializes the timer for PWM output */
    Result InitPWM(const Config& config, const PWMChannelConfig* channels, uint8_t num_channels);

    /** @brief Starts PWM output on configured channels */
    Result StartPWM();

    /** @brief Stops PWM output on configured channels */
    Result StopPWM();

    /** @brief Sets the pulse width for a specific channel */
    Result SetPWMPulse(uint32_t channel, uint32_t pulse);

    class Impl;

  private:
    Impl* pimpl_;
};

} // namespace uvos

#endif
