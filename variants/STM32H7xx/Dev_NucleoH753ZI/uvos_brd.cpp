#include "uvos_brd.h"

using namespace uvos;

#define UVS_LED_PORT UVS_GPIOE
#define UVS_LED_PIN 1

#define UVS_TEST_POINT_PORT UVS_GPIOB
#define UVS_TEST_POINT_PIN 14

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
    led.pin.port       = UVS_LED_PORT;
    led.pin.pin        = UVS_LED_PIN;
    led.mode           = UVS_GPIO_MODE_OUTPUT_PP;
    testpoint.pin.port = UVS_TEST_POINT_PORT;
    testpoint.pin.pin  = UVS_TEST_POINT_PIN;
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

#if 0 // @TODO

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

#endif // @TODO
    return BoardVersion::UVOS_BOARD;
}
