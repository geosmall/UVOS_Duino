#pragma once

#include "uvos.h"

namespace uvos
{
/**
   @brief This is the higher-level interface for the UVOS board. \n 
    All basic peripheral configuration/initialization is setup here. \n

   @ingroup boards
*/
class UVOSboard
{
  public:
    UVOSboard() {}
    ~UVOSboard() {}

    /** This function used to provide a pre-initialization configuraiton 
     *  it has since been deprecated, and does nothing.
     */
    void Configure();

    /** 
    Initializes the UVOS board and the following peripherals:
    SDRAM, QSPI, 24-bit 48kHz Audio via AK4556, Internal USB,
    as well as the built-in LED and Testpoint.

    ADCs, DACs, and other special peripherals (such as I2C, SPI, etc.)
    can be initialized using their specific initializers within libuvos
    for a specific application.
    */
    void Init(bool boost = false);

    /** 
    Deinitializes all peripherals automatically handled by `Init`.
    */
    void DeInit();

    /** 
    Wait some ms before going on.
    \param del Delay time in ms.
    */
    void DelayMs(size_t del);

    /** 
    Returns the gpio_pin corresponding to the index 0-31.
    For the given GPIO on the UVOS board (labeled 1-32 in docs).
    */
    static uvs_gpio_pin GetPin(uint8_t pin_idx);

    /** Sets the state of the built in LED
     */
    void SetLed(bool state);

    /** Sets the state of the test point near pin 10
     */
    void SetTestPoint(bool state);

#if 0 // gls

    /** Print formatted debug log message
     */
    template <typename... VA>
    static void Print(const char* format, VA... va)
    {
        Log::Print(format, va...);
    }

    /** Print formatted debug log message with automatic line termination
    */
    template <typename... VA>
    static void PrintLine(const char* format, VA... va)
    {
        Log::PrintLine(format, va...);
    }

    /** Start the logging session. Optionally wait for terminal connection before proceeding.
    */
    static void StartLog(bool wait_for_pc = false)
    {
        Log::StartLog(wait_for_pc);
    }

#endif // gls

    // While the library is still in heavy development, most of the
    // configuration handles will remain public.
    // QSPIHandle         qspi;
    // QSPIHandle::Config qspi_config;
    // SdramHandle        sdram_handle; /**< & */
    // AudioHandle        audio_handle; /**< & */
    // AdcHandle          adc;          /**< & */
    // DacHandle          dac;
    UsbHandle          usb_handle; /**< & */
    uvs_gpio           led, testpoint;
    System             system;

    /** Internal indices for UVOSboard-equivalent devices 
     *  This shouldn't have any effect on user-facing code,
     *  and only needs to be checked to properly initialize
     *  the onboard-circuits.
    */
    enum class BoardVersion
    {
        /** UVOS board Rev4
         *  This is the original UVOS board */
        UVOS_BOARD,
        /** UVOS board 1.1 (aka UVOS board Rev5)
         *  This is a pin-compatible version of the UVOS board
         *  that uses the WM8731 codec instead of the AK4430 */
        UVOS_BOARD_1_1,
        /** UVOS board 2 DFM is a software compatible version of the
         *  original UVOS board that has improvements for manufacturing,
         *  as well as an improved audio codec (PCM3060)
         */
        UVOS_BOARD_2_DFM,
    };

    /** Returns the BoardVersion detected during intiialization */
    BoardVersion CheckBoardVersion();


  private:
    /** Local shorthand for debug log destination
    */
    // using Log = Logger<LOGGER_INTERNAL>;

    // void ConfigureQspi();
    // void ConfigureAudio();
    // void ConfigureAdc();
    // void ConfigureDac();
    //void     ConfigureI2c();
    // float callback_rate_;

    // SaiHandle sai_1_handle_;
};

/** seed namespace contains pinout constants for addressing 
 * the pins on the UVOS board SOM.
 */
namespace seed
{
    /** Constant Pinout consts */
    constexpr Pin D0  = Pin(PORTB, 12);
    constexpr Pin D1  = Pin(PORTC, 11);
    constexpr Pin D2  = Pin(PORTC, 10);
    constexpr Pin D3  = Pin(PORTC, 9);
    constexpr Pin D4  = Pin(PORTC, 8);
    constexpr Pin D5  = Pin(PORTD, 2);
    constexpr Pin D6  = Pin(PORTC, 12);
    constexpr Pin D7  = Pin(PORTG, 10);
    constexpr Pin D8  = Pin(PORTG, 11);
    constexpr Pin D9  = Pin(PORTB, 4);
    constexpr Pin D10 = Pin(PORTB, 5);
    constexpr Pin D11 = Pin(PORTB, 8);
    constexpr Pin D12 = Pin(PORTB, 9);
    constexpr Pin D13 = Pin(PORTB, 6);
    constexpr Pin D14 = Pin(PORTB, 7);
    constexpr Pin D15 = Pin(PORTC, 0);
    constexpr Pin D16 = Pin(PORTA, 3);
    constexpr Pin D17 = Pin(PORTB, 1);
    constexpr Pin D18 = Pin(PORTA, 7);
    constexpr Pin D19 = Pin(PORTA, 6);
    constexpr Pin D20 = Pin(PORTC, 1);
    constexpr Pin D21 = Pin(PORTC, 4);
    constexpr Pin D22 = Pin(PORTA, 5);
    constexpr Pin D23 = Pin(PORTA, 4);
    constexpr Pin D24 = Pin(PORTA, 1);
    constexpr Pin D25 = Pin(PORTA, 0);
    constexpr Pin D26 = Pin(PORTD, 11);
    constexpr Pin D27 = Pin(PORTG, 9);
    constexpr Pin D28 = Pin(PORTA, 2);
    constexpr Pin D29 = Pin(PORTB, 14);
    constexpr Pin D30 = Pin(PORTB, 15);

    /** Analog pins share same pins as digital pins */
    constexpr Pin A0  = D15;
    constexpr Pin A1  = D16;
    constexpr Pin A2  = D17;
    constexpr Pin A3  = D18;
    constexpr Pin A4  = D19;
    constexpr Pin A5  = D20;
    constexpr Pin A6  = D21;
    constexpr Pin A7  = D22;
    constexpr Pin A8  = D23;
    constexpr Pin A9  = D24;
    constexpr Pin A10 = D25;
    constexpr Pin A11 = D28;

    /** Pins unique to UVOS board 2 DFM */
    constexpr Pin D31 = Pin(PORTC, 2);
    constexpr Pin D32 = Pin(PORTC, 3);

    /** Analog Pin alias */
    constexpr Pin A12 = D31;
    constexpr Pin A13 = D32;
} // namespace seed

} // namespace uvos
