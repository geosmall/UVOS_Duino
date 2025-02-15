#include "uvos_brd.h"
#include "dev/pwm_output.h"

static void Error_Handler()
{
    asm("bkpt 255");
    while(1) {}
}

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hardware
UVOSboard hardware;

using TimPeriph = TimerHandle::Config::Peripheral;

int main(void)
{
    // Declare a variable to store the state we want to set for the LED.
    bool led_state;
    led_state = true;

    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hardware.Configure();
    hardware.Init();

    // Define initial pulse width in microseconds
    uint32_t init_pulse_servo = 1500; // Neutral position for servos
    uint32_t init_pulse_esc   = 1000; // Minimum throttle for ESCs

    // Define servo outputs (S1-S6)
    static PWMOutputChannel servo_outputs[] = {
        // S1 and S2 on TIM3
        {TimPeriph::TIM_3, TimChannel::CH_3, Pin(PORTB, 0), init_pulse_servo, TimPolarity::HIGH, GPIO_AF2_TIM3},
        {TimPeriph::TIM_3, TimChannel::CH_4, Pin(PORTB, 1), init_pulse_servo, TimPolarity::HIGH, GPIO_AF2_TIM3},
        // S3-S6 on TIM5
        {TimPeriph::TIM_5, TimChannel::CH_1, Pin(PORTA, 0), init_pulse_servo, TimPolarity::HIGH, GPIO_AF2_TIM5},
        {TimPeriph::TIM_5, TimChannel::CH_2, Pin(PORTA, 1), init_pulse_servo, TimPolarity::HIGH, GPIO_AF2_TIM5},
        {TimPeriph::TIM_5, TimChannel::CH_3, Pin(PORTA, 2), init_pulse_servo, TimPolarity::HIGH, GPIO_AF2_TIM5},
        {TimPeriph::TIM_5, TimChannel::CH_4, Pin(PORTA, 3), init_pulse_servo, TimPolarity::HIGH, GPIO_AF2_TIM5},
    };
    constexpr size_t NUM_SERVO_OUTPUTS = ARRAYLEN(servo_outputs);

    // Define ESC outputs (S7-S10)
    static PWMOutputChannel esc_outputs[] = {
        // S7-S10 on TIM4
        {TimPeriph::TIM_4, TimChannel::CH_1, Pin(PORTD, 12), init_pulse_esc, TimPolarity::HIGH, GPIO_AF2_TIM4},
        {TimPeriph::TIM_4, TimChannel::CH_2, Pin(PORTD, 13), init_pulse_esc, TimPolarity::HIGH, GPIO_AF2_TIM4},
        {TimPeriph::TIM_4, TimChannel::CH_3, Pin(PORTD, 14), init_pulse_esc, TimPolarity::HIGH, GPIO_AF2_TIM4},
        {TimPeriph::TIM_4, TimChannel::CH_4, Pin(PORTD, 15), init_pulse_esc, TimPolarity::HIGH, GPIO_AF2_TIM4},
        {TimPeriph::TIM_15, TimChannel::CH_1, Pin(PORTE, 5), init_pulse_esc, TimPolarity::HIGH, GPIO_AF4_TIM15},
        {TimPeriph::TIM_15, TimChannel::CH_2, Pin(PORTE, 6), init_pulse_esc, TimPolarity::HIGH, GPIO_AF4_TIM15},
    };
    constexpr size_t NUM_ESC_OUTPUTS = ARRAYLEN(esc_outputs);

    // Create and initialize servo PWMOutput object,
    // check for initialization errors
    PWMOutput Servo_pwm(servo_outputs, NUM_SERVO_OUTPUTS, 50); // 50 Hz for servos
    if (Servo_pwm.Init() != PWMOutput::Result::OK) Error_Handler();

    // Create and initialize servo PWMOutput object,
    // check for initialization errors
    PWMOutput ESC_pwm(esc_outputs, NUM_ESC_OUTPUTS, 500); // 500 Hz for ESCs
    if (ESC_pwm.Init() != PWMOutput::Result::OK) Error_Handler();

    // Example usage: Set pulse widths
    // Set servo outputs to different positions
    Servo_pwm.SetPulseWidth(0, 1000); // Servo S1 to 1000 microseconds
    Servo_pwm.SetPulseWidth(1, 1300); // Servo S2 to 1300 microseconds
    Servo_pwm.SetPulseWidth(2, 1500); // Servo S3 to 1500 microseconds
    Servo_pwm.SetPulseWidth(3, 1700); // Servo S4 to 1700 microseconds
    Servo_pwm.SetPulseWidth(4, 1900); // Servo S5 to 1900 microseconds
    Servo_pwm.SetPulseWidth(5, 2000); // Servo S6 to 2000 microseconds

    // Set ESC outputs to different throttle levels
    ESC_pwm.SetPulseWidth(0, 1100); // ESC S7 to 1100 microseconds
    ESC_pwm.SetPulseWidth(1, 1200); // ESC S8 to 1200 microseconds
    ESC_pwm.SetPulseWidth(2, 1300); // ESC S9 to 1300 microseconds
    ESC_pwm.SetPulseWidth(3, 1400); // ESC S10 to 1400 microseconds
    ESC_pwm.SetPulseWidth(4, 1500); // ESC S11 to 1500 microseconds
    ESC_pwm.SetPulseWidth(5, 1600); // ESC S12 to 1600 microseconds

    // Loop forever
    for(;;)
    {
        // Set the onboard LED
        hardware.SetLed(led_state);

        // Toggle the LED state for the next time around.
        led_state = !led_state;

        // Wait 500ms
        System::Delay(500);
    }
}
