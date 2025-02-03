#pragma once

#include "uvos_core.h"

#ifdef __cplusplus

namespace uvos
{
/** @brief General Purpose I/O control 
 *  @details peripheral control over a single GPIO
 *  @ingroup peripheral
 *   
 *  Button Input (with internal Pullup resistor)
 *  @include GPIO_Input.cpp 
 * 
 *  Output Example (perfect for blinking an LED)
 *  @include GPIO_Output.cpp
 * 
*/
class GPIO
{
  public:
    /** @brief Mode of operation for the specified GPIO */
    enum class Mode
    {
        INPUT,     /**< Input for reading state of pin */
        OUTPUT,    /**< Output w/ push-pull configuration */
        OUTPUT_OD, /**< Output w/ open-drain configuration */
        ANALOG,    /**< Analog for connection to ADC or DAC peripheral */
        AF_PP,     /**< Alternate Function Push-Pull */
        AF_OD,     /**< Alternate Function Open-Drain */
        // --- Interrupt-capable modes (new):
        INPUT_IT_RISING,         /**< Input with interrupt on rising edge */
        INPUT_IT_FALLING,        /**< Input with interrupt on falling edge */
        INPUT_IT_RISING_FALLING  /**< Input with interrupt on both edges */
    };

    /** @brief Configures whether an internal Pull up or Pull down resistor is used. 
     *  
     * Internal Pull up/down resistors are typically 40k ohms, and will be between
     * 30k and 50k
     * 
     * When the Pin is configured in Analog mode, the pull up/down resistors are
     * disabled by hardware. 
     */
    enum class Pull
    {
        NOPULL,   /**< No pull up resistor */
        PULLUP,   /**< Internal pull up enabled */
        PULLDOWN, /**< Internal pull down enabled*/
    };

    /** @brief Output speed controls the drive strength, and slew rate of the pin */
    enum class Speed
    {
        LOW,
        MEDIUM,
        HIGH,
        VERY_HIGH,
    };

    /** @brief Configuration for a given GPIO */
    struct Config
    {
        Pin      pin;
        Mode     mode;
        Pull     pull;
        Speed    speed;
        uint32_t alternate; /**< Alternate function number */

        /** Constructor with no arguments will prepare an invalid GPIO set as
         *  an input, with no pullup. 
         */
        Config()
        : pin(), mode(Mode::INPUT), pull(Pull::NOPULL), speed(Speed::LOW), alternate(0)
        {
        }
    };

    GPIO() {}

    /** @brief Initialize the GPIO from a Config struct 
     *  @param cfg reference to a Config struct populated with the desired settings
    */
    void Init(const Config &cfg);

    /** @brief Initialize the GPIO with a Configuration struct, and explicit pin 
     *  @param p Pin specifying the physical connection on the hardware
     *  @param cfg reference to a Config struct populated with the desired settings. 
     *         Config::pin will be overwritten
    */
    void Init(Pin p, const Config &cfg);

    /** @brief Explicitly initialize all configuration for the GPIO 
     *  @param p Pin specifying the physical connection on the hardware
     *  @param m Mode specifying the behavior of the GPIO (input, output, etc.). Defaults to Mode::INPUT
     *  @param pu Pull up/down state for the GPIO. Defaults to Pull::NOPULL
     *  @param sp Speed setting for drive strength/slew rate. Defaults to Speed::LOW
     *  @param alt Alternate function number. Defaults to 0
    */
    void Init(Pin     p,
              Mode    m  = Mode::INPUT,
              Pull    pu = Pull::NOPULL,
              Speed   sp = Speed::LOW,
              uint32_t alt = 0);

    /** @brief Deinitializes the GPIO pin */
    void DeInit();

    /** @brief Reads the state of the GPIO.
     *  @return State of the GPIO unless Mode is set to Mode::Analog, then always false
     */
    bool Read();

    /** @brief Changes the state of the GPIO hardware when configured as an OUTPUT. 
     *  @param state setting true writes an output HIGH, while setting false writes an output LOW.
     */
    void Write(bool state);

    /** @brief flips the current state of the GPIO. 
     *  If it was HIGH, it will go LOW, and vice versa.
     */
    void Toggle();

    /** Return a reference to the internal Config struct */
    Config &GetConfig() { return cfg_; }

    /* --- New Interrupt Callback Support --- */

    /** @brief Type for the interrupt callback function.
     *
     * This is a C function pointer that takes no parameters and returns void.
     */
    typedef void (*InterruptCallback)(void);

    /** @brief Register an interrupt callback for EXTI events.
     *
     * When the GPIO is configured in an interrupt mode (INPUT_IT_RISING, INPUT_IT_FALLING,
     * or INPUT_IT_RISING_FALLING), this callback will be invoked when an EXTI event occurs.
     */
    void SetInterruptCallback(InterruptCallback cb);

    /** @brief Dispatch EXTI callback to the appropriate GPIO instance.
     *
     * This function is called by the global HAL_GPIO_EXTI_Callback.
     */
    static void DispatchExtiCallback(uint16_t GPIO_Pin);

  private:
    /** This will internally be cast to the 
     *  STM32H7 GPIO_Typedef* type, which 
     *  is just the base address to the
     *  specified GPIO register. 
     * 
     *  This prevents us needing to have an internal
     *  Impl class just to store the GPIO_Typedef* 
     */
    uint32_t* GetGPIOBaseRegister();

    /** Internal copy of the Configuration of the given pin */
    Config cfg_;

    /** Internal pointer to base address of relavent GPIO register */
    uint32_t* port_base_addr_;

    /* --- New Private Members for Interrupt Support --- */

    /** User-registered interrupt callback for EXTI events */
    InterruptCallback interruptCallback_;

    /** @brief Handle the interrupt event (invokes the user callback if set)
     *
     * This function is called internally by DispatchExtiCallback.
     */
    void HandleInterrupt();
};

} // namespace uvos


/** @ingroup peripheral
 *  @addtogroup DEPRECATED-OLD-GPIO
 * 
 *  @brief Deprecated C API for GPIO is staying in place for a 
 *  few versions to support backwards compatibility.
 * 
 *  This should not be used for anything new.
 *  @deprecated These should only be used for casting to configs, and are planned to be removed in a future version.
 *  @{
 */
extern "C"
{
    /** General Purpose IO driver */

    /** Sets the mode of the GPIO */
    typedef enum
    {
        UVS_GPIO_MODE_INPUT,     /**< & */
        UVS_GPIO_MODE_OUTPUT_PP, /**< Push-Pull */
        UVS_GPIO_MODE_OUTPUT_OD, /**< Open-Drain */
        UVS_GPIO_MODE_ANALOG,    /**< & */
        UVS_GPIO_MODE_LAST,      /**< & */
    } uvs_gpio_mode;

    /** Configures whether an internal Pull up or Pull down resistor is used */
    typedef enum
    {
        UVS_GPIO_NOPULL,   /**< & */
        UVS_GPIO_PULLUP,   /**< & */
        UVS_GPIO_PULLDOWN, /**< & */
    } uvs_gpio_pull;

    /** Struct for holding the pin, and configuration */
    typedef struct
    {
        uvs_gpio_pin  pin;  /**< & */
        uvs_gpio_mode mode; /**< & */
        uvs_gpio_pull pull; /**< & */
    } uvs_gpio;

    /** Initializes the gpio with the settings configured. 
    \param *p Pin pointer
    */
    void uvs_gpio_init(const uvs_gpio *p);

    /** Deinitializes the gpio pin 
    \param *p Pin pointer
     */
    void uvs_gpio_deinit(const uvs_gpio *p);

    /** 
    Reads the state of the gpio pin
    \param *p Pin pointer 
    \return 1 if the pin is HIGH, and 0 if the pin is LOW */
    uint8_t uvs_gpio_read(const uvs_gpio *p);

    /** 
    Writes the state to the gpio pin
    Pin will be set to 3v3 when state is 1, and 0V when state is 0
    \param *p Pin pointer
    \param state State to write
    */
    void uvs_gpio_write(const uvs_gpio *p, uint8_t state);

    /** Toggles the state of the pin so that it is not at the same state as it was previously.
    \param *p Pin pointer
     */
    void uvs_gpio_toggle(const uvs_gpio *p);
    /**@} */
}

#endif /* __cplusplus */
