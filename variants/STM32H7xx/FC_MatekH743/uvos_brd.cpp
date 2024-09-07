#include "uvos_brd.h"

using namespace uvos;

#define SEED_LED_PORT UVS_GPIOE
#define SEED_LED_PIN 3

#define SEED_TEST_POINT_PORT UVS_GPIOE
#define SEED_TEST_POINT_PIN 4

#ifndef SEED_REV2
const uvs_gpio_pin seedgpio[33] = {
    // GPIO 1-8
    //{UVS_GPIOA, 8}, // removed on Rev4
    {UVS_GPIOB, 12},
    {UVS_GPIOC, 11},
    {UVS_GPIOC, 10},
    {UVS_GPIOC, 9},
    {UVS_GPIOC, 8},
    {UVS_GPIOD, 2},
    {UVS_GPIOC, 12},
    // GPIO 9-16
    {UVS_GPIOG, 10},
    {UVS_GPIOG, 11},
    {UVS_GPIOB, 4},
    {UVS_GPIOB, 5},
    {UVS_GPIOB, 8},
    {UVS_GPIOB, 9},
    {UVS_GPIOB, 6},
    {UVS_GPIOB, 7},
    // GPIO 17-24
    {UVS_GPIOC, 0},
    {UVS_GPIOA, 3},
    {UVS_GPIOB, 1},
    {UVS_GPIOA, 7},
    {UVS_GPIOA, 6},
    {UVS_GPIOC, 1},
    {UVS_GPIOC, 4},
    {UVS_GPIOA, 5},
    // GPIO 25-31
    {UVS_GPIOA, 4},
    {UVS_GPIOA, 1},
    {UVS_GPIOA, 0},
    {UVS_GPIOD, 11},
    {UVS_GPIOG, 9},
    {UVS_GPIOA, 2},
    {UVS_GPIOB, 14},
    {UVS_GPIOB, 15},

    // Seed2DFM exclusive pins
    {UVS_GPIOC, 2},
    {UVS_GPIOC, 3},
};
#else
const uvs_gpio_port seed_ports[32] = {
    UVS_GPIOA, UVS_GPIOB, UVS_GPIOC, UVS_GPIOC, UVS_GPIOC, UVS_GPIOC, UVS_GPIOD,
    UVS_GPIOC, UVS_GPIOG, UVS_GPIOG, UVS_GPIOB, UVS_GPIOB, UVS_GPIOB, UVS_GPIOB,
    UVS_GPIOB, UVS_GPIOB, UVS_GPIOC, UVS_GPIOA, UVS_GPIOA, UVS_GPIOB, UVS_GPIOA,
    UVS_GPIOA, UVS_GPIOC, UVS_GPIOC, UVS_GPIOA, UVS_GPIOA, UVS_GPIOA, UVS_GPIOD,
    UVS_GPIOG, UVS_GPIOA, UVS_GPIOB, UVS_GPIOB,
};

const uint8_t seed_pins[32] = {
    8, 12, 11, 10, 9, 8, 7, 12, 10, 11, 4, 5,  8, 9, 6,  7,
    0, 1,  3,  1,  7, 6, 1, 5,  5,  4,  0, 11, 9, 2, 14, 15,
};

const uvs_gpio_pin seedgpio[32] = {
    {seed_ports[0], seed_pins[0]},   {seed_ports[1], seed_pins[1]},
    {seed_ports[2], seed_pins[2]},   {seed_ports[3], seed_pins[3]},
    {seed_ports[4], seed_pins[4]},   {seed_ports[5], seed_pins[5]},
    {seed_ports[6], seed_pins[6]},   {seed_ports[7], seed_pins[7]},
    {seed_ports[8], seed_pins[8]},   {seed_ports[9], seed_pins[9]},
    {seed_ports[10], seed_pins[10]}, {seed_ports[11], seed_pins[11]},
    {seed_ports[12], seed_pins[12]}, {seed_ports[13], seed_pins[13]},
    {seed_ports[14], seed_pins[14]}, {seed_ports[15], seed_pins[15]},
    {seed_ports[16], seed_pins[16]}, {seed_ports[17], seed_pins[17]},
    {seed_ports[18], seed_pins[18]}, {seed_ports[19], seed_pins[19]},
    {seed_ports[20], seed_pins[20]}, {seed_ports[21], seed_pins[21]},
    {seed_ports[22], seed_pins[22]}, {seed_ports[23], seed_pins[23]},
    {seed_ports[24], seed_pins[24]}, {seed_ports[25], seed_pins[25]},
    {seed_ports[26], seed_pins[26]}, {seed_ports[27], seed_pins[27]},
    {seed_ports[28], seed_pins[28]}, {seed_ports[29], seed_pins[29]},
    {seed_ports[30], seed_pins[30]}, {seed_ports[31], seed_pins[31]},
};
#endif

// Public Initialization

/** Vestigial function body for old function
 *  This is no longer in use.
 */
void UVOSboard::Configure() {}

void UVOSboard::Init(bool boost)
{
    //uvs_system_init();
    System::Config syscfg;
    boost ? syscfg.Boost() : syscfg.Defaults();

#if 0 // gls
    ConfigureQspi();
#endif // gls
    // Configure the built-in GPIOs.
    led.pin.port       = SEED_LED_PORT;
    led.pin.pin        = SEED_LED_PIN;
    led.mode           = UVS_GPIO_MODE_OUTPUT_PP;
    testpoint.pin.port = SEED_TEST_POINT_PORT;
    testpoint.pin.pin  = SEED_TEST_POINT_PIN;
    testpoint.mode     = UVS_GPIO_MODE_OUTPUT_PP;


    auto memory       = System::GetProgramMemoryRegion();
    auto boot_version = System::GetBootloaderVersion();

    if(boot_version == System::BootInfo::Version::LT_v6_0
       && memory != System::MemoryRegion::INTERNAL_FLASH)
    {
        syscfg.skip_clocks = true;
    }

    system.Init(syscfg);

#if 0 // gls
    if(memory != System::MemoryRegion::QSPI)
        qspi.Init(qspi_config);
#endif // gls

    if(boot_version != System::BootInfo::Version::LT_v6_0
       || (boot_version == System::BootInfo::Version::LT_v6_0
           && memory == System::MemoryRegion::INTERNAL_FLASH))
    {
        uvs_gpio_init(&led);
        uvs_gpio_init(&testpoint);
#if 0 // gls
        sdram_handle.Init();
#endif // gls
    }

#if 0 // gls
    ConfigureAudio();

    callback_rate_ = AudioSampleRate() / AudioBlockSize();
    // Due to the added 16kB+ of flash usage,
    // and the fact that certain breakouts use
    // both; USB won't be initialized by the
    // SEED file.
    //usb_handle.Init(UsbHandle::FS_INTERNAL);
#endif // gls
}

void UVOSboard::DeInit()
{
    // This is intended to be used by the bootloader, but
    // we don't want to reinitialize pretty much anything in the
    // target application, so...
    // qspi.DeInit();
    // sdram_handle.DeInit();
    // uvs_gpio_deinit(&led);
    // uvs_gpio_deinit(&testpoint);

    // uvs_gpio_pin codec_reset_pin;
    // codec_reset_pin = {UVS_GPIOB, 11};
    // // Perhaps a bit unnecessary, but maybe we'll make
    // // this non-static at some point
    // Ak4556::DeInit(codec_reset_pin);
    // audio_handle.DeInit();

    system.DeInit();
}

uvs_gpio_pin UVOSboard::GetPin(uint8_t pin_idx)
{
    uvs_gpio_pin p;
    pin_idx = pin_idx < sizeof(seedgpio) / sizeof(seedgpio[0]) ? pin_idx : 0;
#ifndef SEED_REV2
    p = seedgpio[pin_idx];
#else
    p = {seed_ports[pin_idx], seed_pins[pin_idx]};
#endif
    return p;
}

void UVOSboard::DelayMs(size_t del)
{
    system.Delay(del);
}

void UVOSboard::SetLed(bool state)
{
    uvs_gpio_write(&led, state);
}

void UVOSboard::SetTestPoint(bool state)
{
    uvs_gpio_write(&testpoint, state);
}

UVOSboard::BoardVersion UVOSboard::CheckBoardVersion()
{
    /** Version Checks:
     *  * Fall through is UVOS board v1 (aka UVOS board rev4)
     *  * PD3 tied to gnd is UVOS board v1.1 (aka UVOS board rev5)
     *  * PD4 tied to gnd reserved for future hardware
     */

    /** Initialize GPIO */
    GPIO s2dfm_gpio, seed_1_1_gpio;
    Pin  seed_1_1_pin(PORTD, 3);
    Pin  s2dfm_pin(PORTD, 4);
    seed_1_1_gpio.Init(seed_1_1_pin, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    s2dfm_gpio.Init(s2dfm_pin, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    /** Perform Check */
    if(!seed_1_1_gpio.Read())
        return BoardVersion::UVOS_BOARD_1_1;
    else if(!s2dfm_gpio.Read())
        return BoardVersion::UVOS_BOARD_2_DFM;
    else
        return BoardVersion::UVOS_BOARD;
}
