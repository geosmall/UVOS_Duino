/****************************************************************
 * HardwareSerial.ino – Minimal test for HardwareSerial.h/.cpp
 * Hardware: Nucleo-H753ZI  (USART3 on PD8/PD9)
 ****************************************************************/
#include "uvos_brd.h"
#include "Dps310.h"

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

static constexpr size_t DMA_BUF_SIZE = 256;   // DMA rx buffer size
static uint8_t DMA_BUFFER_MEM_SECTION HardwareSerial_rx_buf[DMA_BUF_SIZE] = {0};

// Build full HardwareSerial::Config with nested lambda
static const HardwareSerial::Config serial_cfg = [] {
    HardwareSerial::Config c{};
    // UART settings
    c.uart_config.periph        = UartHandler::Config::Peripheral::USART_3;
    c.uart_config.mode          = UartHandler::Config::Mode::TX_RX;
    c.uart_config.pin_config.tx = Pin(PORTD, 8);
    c.uart_config.pin_config.rx = Pin(PORTD, 9);
    // DMA buffer
    c.dma_buf      = HardwareSerial_rx_buf;
    c.dma_buf_size = DMA_BUF_SIZE;
    return c;
}();

// Wrapper instance
HardwareSerial Serial(serial_cfg);

// static TwoWire::Config defaultWireCfg = []{
//     TwoWire::Config cfg{};
//     // select I2C2 and default pins; set default speed to 400 kHz, Master mode
//     cfg.i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_2;
//     cfg.i2c_config.pin_config.scl = Pin(PORTB, 10);
//     cfg.i2c_config.pin_config.sda = Pin(PORTB, 11);
//     cfg.i2c_config.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
//     cfg.i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
//     return cfg;
// }();
// TwoWire Wire(defaultWireCfg);

// Dps310 Opject
Dps310 Dps310PressureSensor = Dps310();

int main(void)
{
    // Configure and Initialize the UVOS board
    // These are separate to allow reconfiguration of any of the internal
    // components before initialization.
    hw.Configure();
    hw.Init();

    Serial.begin(115200);
    while (!Serial);

    Serial.println("Example i2c_background.ino");

    //Call begin to initialize Dps310PressureSensor
    //The parameter 0x76 is the bus address. The default address is 0x77 and does not need to be given.
    Dps310PressureSensor.begin(Wire, 0x76);
    //Use the commented line below instead to use the default I2C address.
    // Dps310PressureSensor.begin(Wire);

    //temperature measure rate (value from 0 to 7)
    //2^temp_mr temperature measurement results per second
    int16_t temp_mr = 2;
    //temperature oversampling rate (value from 0 to 7)
    //2^temp_osr internal temperature measurements per result
    //A higher value increases precision
    int16_t temp_osr = 2;
    //pressure measure rate (value from 0 to 7)
    //2^prs_mr pressure measurement results per second
    int16_t prs_mr = 2;
    //pressure oversampling rate (value from 0 to 7)
    //2^prs_osr internal pressure measurements per result
    //A higher value increases precision
    int16_t prs_osr = 2;
    //startMeasureBothCont enables background mode
    //temperature and pressure ar measured automatically
    //High precision and hgh measure rates at the same time are not available.
    //Consult Datasheet (or trial and error) for more information
    int16_t ret = Dps310PressureSensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
    //Use one of the commented lines below instead to measure only temperature or pressure
    //int16_t ret = Dps310PressureSensor.startMeasureTempCont(temp_mr, temp_osr);
    //int16_t ret = Dps310PressureSensor.startMeasurePressureCont(prs_mr, prs_osr);


    if (ret != 0) {
        Serial.print("Init FAILED! ret = ");
        Serial.println(ret);
    } else {
        Serial.println("Init complete!");
    }

    for (;;) { // Loop forever

        uint8_t pressureCount = 20;
        float pressure[pressureCount];
        uint8_t temperatureCount = 20;
        float temperature[temperatureCount];

        //This function writes the results of continuous measurements to the arrays given as parameters
        //The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
        //After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
        //Note: The Dps310 cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results
        int16_t ret = Dps310PressureSensor.getContResults(temperature, temperatureCount, pressure, pressureCount);

        if (ret != 0) {
            Serial.println();
            Serial.println();
            Serial.print("FAIL! ret = ");
            Serial.println(ret);
        } else {
            Serial.println();
            Serial.println();
            Serial.print(temperatureCount);
            Serial.println(" temperature values found: ");
            for (int16_t i = 0; i < temperatureCount; i++) {
                Serial.print(temperature[i]);
                Serial.println(" degrees of Celsius");
            }

            Serial.println();
            Serial.print(pressureCount);
            Serial.println(" pressure values found: ");
            for (int16_t i = 0; i < pressureCount; i++) {
                Serial.print(pressure[i]);
                Serial.println(" Pascal");
            }
        }

        //Wait some time, so that the Dps310 can refill its buffer
        delay(10000);
    }

    return 0;
}
