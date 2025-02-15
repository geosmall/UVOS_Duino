#include "gpio.h"
#include "stm32h7xx_hal.h"
#include <string.h> // For memset

using namespace uvos;

//------------------------------------------------------------------------------
// Static registry for mapping EXTI lines (0–15) to GPIO instances.
//------------------------------------------------------------------------------
static GPIO* extiHandlers[16] = { nullptr };

void GPIO::Init(const Config &cfg)
{
    /** Copy Config */
    cfg_ = cfg;

    if(!cfg_.pin.IsValid())
        return;

    GPIO_InitTypeDef ginit;
    memset(&ginit, 0, sizeof(ginit));

    switch(cfg_.mode)
    {
        case Mode::OUTPUT: ginit.Mode = GPIO_MODE_OUTPUT_PP; break;
        case Mode::OUTPUT_OD: ginit.Mode = GPIO_MODE_OUTPUT_OD; break;
        case Mode::ANALOG: ginit.Mode = GPIO_MODE_ANALOG; break;
        case Mode::AF_PP: ginit.Mode = GPIO_MODE_AF_PP; break;
        case Mode::AF_OD: ginit.Mode = GPIO_MODE_AF_OD; break;
        // --- Interrupt-capable modes (new):
        case Mode::INPUT_IT_RISING: ginit.Mode = GPIO_MODE_IT_RISING; break;
        case Mode::INPUT_IT_FALLING: ginit.Mode = GPIO_MODE_IT_FALLING; break;
        case Mode::INPUT_IT_RISING_FALLING: ginit.Mode = GPIO_MODE_IT_RISING_FALLING; break;
        case Mode::INPUT:
        default: ginit.Mode = GPIO_MODE_INPUT; break;
    }

    switch(cfg_.pull)
    {
        case Pull::PULLUP: ginit.Pull = GPIO_PULLUP; break;
        case Pull::PULLDOWN: ginit.Pull = GPIO_PULLDOWN; break;
        case Pull::NOPULL:
        default: ginit.Pull = GPIO_NOPULL;
    }

    switch(cfg_.speed)
    {
        case Speed::VERY_HIGH: ginit.Speed = GPIO_SPEED_FREQ_VERY_HIGH; break;
        case Speed::HIGH: ginit.Speed = GPIO_SPEED_FREQ_HIGH; break;
        case Speed::MEDIUM: ginit.Speed = GPIO_SPEED_FREQ_MEDIUM; break;
        case Speed::LOW:
        default: ginit.Speed = GPIO_SPEED_FREQ_LOW;
    }

    if(cfg_.mode == Mode::AF_PP || cfg_.mode == Mode::AF_OD)
    {
        ginit.Alternate = cfg_.alternate;
    }
    else
    {
        ginit.Alternate = 0;
    }

    port_base_addr_ = GetGPIOBaseRegister();
    /** Start Relevant GPIO clock */
    switch(cfg_.pin.port)
    {
        case PORTA: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
        case PORTB: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
        case PORTC: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
        case PORTD: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        case PORTE: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
        case PORTF: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
        case PORTG: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
        case PORTH: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
        case PORTI: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
        case PORTJ: __HAL_RCC_GPIOJ_CLK_ENABLE(); break;
        case PORTK: __HAL_RCC_GPIOK_CLK_ENABLE(); break;
        default: break;
    }
    /** Set pin based on stm32 schema */
    ginit.Pin = (1 << cfg_.pin.pin);
    HAL_GPIO_Init(reinterpret_cast<GPIO_TypeDef *>(port_base_addr_), &ginit);

    // If configured for interrupts, register this instance and configure NVIC.
    if (cfg_.mode == Mode::INPUT_IT_RISING ||
        cfg_.mode == Mode::INPUT_IT_FALLING ||
        cfg_.mode == Mode::INPUT_IT_RISING_FALLING)
    {
        if (cfg_.pin.pin < 16)
        {
            extiHandlers[cfg_.pin.pin] = this;
        }

        // Determine the correct IRQ for the given pin.
        IRQn_Type irq;
        if (cfg_.pin.pin <= 4)
        {
            switch (cfg_.pin.pin)
            {
                case 0: irq = EXTI0_IRQn; break;
                case 1: irq = EXTI1_IRQn; break;
                case 2: irq = EXTI2_IRQn; break;
                case 3: irq = EXTI3_IRQn; break;
                case 4: irq = EXTI4_IRQn; break;
                default: irq = EXTI0_IRQn; break; // fallback (should not occur)
            }
        }
        else if (cfg_.pin.pin <= 9)
        {
            irq = EXTI9_5_IRQn;
        }
        else  // pins 10 to 15
        {
            irq = EXTI15_10_IRQn;
        }
        HAL_NVIC_SetPriority(irq, 2, 0);
        HAL_NVIC_EnableIRQ(irq);
    }
}

void GPIO::Init(Pin p, const Config &cfg)
{
    /** Copy config */
    cfg_ = cfg;
    /** Overwrite with explicit pin */
    cfg_.pin = p;
    Init(cfg_);
}

void GPIO::Init(Pin p, Mode m, Pull pu, Speed sp, uint32_t alt)
{
    // Populate Config struct, and init with overload
    cfg_.pin       = p;
    cfg_.mode      = m;
    cfg_.pull      = pu;
    cfg_.speed     = sp;
    cfg_.alternate = alt;
    Init(cfg_);
}

void GPIO::DeInit()
{
    if(cfg_.pin.IsValid())
        HAL_GPIO_DeInit(reinterpret_cast<GPIO_TypeDef *>(port_base_addr_), (1 << cfg_.pin.pin));
}

bool GPIO::Read()
{
    return HAL_GPIO_ReadPin(reinterpret_cast<GPIO_TypeDef *>(port_base_addr_),
                              (1 << cfg_.pin.pin)) == GPIO_PIN_SET;
}

void GPIO::Write(bool state)
{
    HAL_GPIO_WritePin(reinterpret_cast<GPIO_TypeDef *>(port_base_addr_),
                      (1 << cfg_.pin.pin),
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void GPIO::Toggle()
{
    HAL_GPIO_TogglePin(reinterpret_cast<GPIO_TypeDef *>(port_base_addr_), (1 << cfg_.pin.pin));
}

uint32_t *GPIO::GetGPIOBaseRegister()
{
    switch(cfg_.pin.port)
    {
        case PORTA: return reinterpret_cast<uint32_t*>(GPIOA);
        case PORTB: return reinterpret_cast<uint32_t*>(GPIOB);
        case PORTC: return reinterpret_cast<uint32_t*>(GPIOC);
        case PORTD: return reinterpret_cast<uint32_t*>(GPIOD);
        case PORTE: return reinterpret_cast<uint32_t*>(GPIOE);
        case PORTF: return reinterpret_cast<uint32_t*>(GPIOF);
        case PORTG: return reinterpret_cast<uint32_t*>(GPIOG);
        case PORTH: return reinterpret_cast<uint32_t*>(GPIOH);
        case PORTI: return reinterpret_cast<uint32_t*>(GPIOI);
        case PORTJ: return reinterpret_cast<uint32_t*>(GPIOJ);
        case PORTK: return reinterpret_cast<uint32_t*>(GPIOK);
        default:    return nullptr;
    }
}

/* Register the user interrupt callback */
void GPIO::SetInterruptCallback(InterruptCallback cb, void *context)
{
    interruptCallback_ = cb;
    interruptCallbackContext_ = context;
}

/* Private method to handle the interrupt event (invokes the user callback if set) */
void GPIO::HandleInterrupt()
{
    if (interruptCallback_)
    {
        interruptCallback_(interruptCallbackContext_);
    }
}

/* Public static method that dispatches EXTI events to the proper GPIO instance */
void GPIO::DispatchExtiCallback(uint16_t GPIO_Pin)
{
    for (uint8_t pin = 0; pin < 16; pin++)
    {
        if (GPIO_Pin & (1 << pin))
        {
            if (extiHandlers[pin] != nullptr)
            {
                extiHandlers[pin]->HandleInterrupt();
            }
        }
    }
}

//------------------------------------------------------------------------------
// EXTI IRQ Handlers (all provided here within the driver).
//------------------------------------------------------------------------------

extern "C" {

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(1 << 0);
}

void EXTI1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(1 << 1);
}

void EXTI2_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(1 << 2);
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(1 << 3);
}

void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(1 << 4);
}

void EXTI9_5_IRQHandler(void)
{
    // Handle EXTI lines 5 to 9.
    for (uint32_t pin = 5; pin <= 9; pin++)
    {
        if (__HAL_GPIO_EXTI_GET_IT(1 << pin) != RESET)
        {
            HAL_GPIO_EXTI_IRQHandler(1 << pin);
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    // Handle EXTI lines 10 to 15.
    for (uint32_t pin = 10; pin <= 15; pin++)
    {
        if (__HAL_GPIO_EXTI_GET_IT(1 << pin) != RESET)
        {
            HAL_GPIO_EXTI_IRQHandler(1 << pin);
        }
    }
}

/**
 * Global HAL callback for EXTI events.
 * This function dispatches the event to our driver via DispatchExtiCallback().
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO::DispatchExtiCallback(GPIO_Pin);
}

} // extern "C"

/* --- Deprecated C API Implementations (unchanged) --- */

extern "C"
{
#include "util/hal_map.h"

    static void start_clock_for_pin(const uvs_gpio *p)
    {
        switch(p->pin.port)
        {
            case UVS_GPIOA: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
            case UVS_GPIOB: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
            case UVS_GPIOC: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
            case UVS_GPIOD: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
            case UVS_GPIOE: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
            case UVS_GPIOF: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
            case UVS_GPIOG: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
            case UVS_GPIOH: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
            case UVS_GPIOI: __HAL_RCC_GPIOI_CLK_ENABLE(); break;
            default: break;
        }
    }

    void uvs_gpio_init(const uvs_gpio *p)
    {
        GPIO_TypeDef *   port;
        GPIO_InitTypeDef ginit;
        switch(p->mode)
        {
            case UVS_GPIO_MODE_INPUT: ginit.Mode = GPIO_MODE_INPUT; break;
            case UVS_GPIO_MODE_OUTPUT_PP:
                ginit.Mode = GPIO_MODE_OUTPUT_PP;
                break;
            case UVS_GPIO_MODE_OUTPUT_OD:
                ginit.Mode = GPIO_MODE_OUTPUT_OD;
                break;
            case UVS_GPIO_MODE_ANALOG: ginit.Mode = GPIO_MODE_ANALOG; break;
            default: ginit.Mode = GPIO_MODE_INPUT; break;
        }
        switch(p->pull)
        {
            case UVS_GPIO_NOPULL: ginit.Pull = GPIO_NOPULL; break;
            case UVS_GPIO_PULLUP: ginit.Pull = GPIO_PULLUP; break;
            case UVS_GPIO_PULLDOWN: ginit.Pull = GPIO_PULLDOWN; break;
            default: ginit.Pull = GPIO_NOPULL; break;
        }
        ginit.Speed = GPIO_SPEED_FREQ_LOW;
        port        = uvs_hal_map_get_port(&p->pin);
        ginit.Pin   = uvs_hal_map_get_pin(&p->pin);
        start_clock_for_pin(p);
        HAL_GPIO_Init(port, &ginit);
    }

    void uvs_gpio_deinit(const uvs_gpio *p)
    {
        GPIO_TypeDef *port;
        uint16_t      pin;
        port = uvs_hal_map_get_port(&p->pin);
        pin  = uvs_hal_map_get_pin(&p->pin);
        HAL_GPIO_DeInit(port, pin);
    }

    uint8_t uvs_gpio_read(const uvs_gpio *p)
    {
        return HAL_GPIO_ReadPin(uvs_hal_map_get_port(&p->pin),
                                uvs_hal_map_get_pin(&p->pin));
    }

    void uvs_gpio_write(const uvs_gpio *p, uint8_t state)
    {
        HAL_GPIO_WritePin(uvs_hal_map_get_port(&p->pin),
                          uvs_hal_map_get_pin(&p->pin),
                          (GPIO_PinState)(state > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }

    void uvs_gpio_toggle(const uvs_gpio *p)
    {
        HAL_GPIO_TogglePin(uvs_hal_map_get_port(&p->pin),
                           uvs_hal_map_get_pin(&p->pin));
    }
}
