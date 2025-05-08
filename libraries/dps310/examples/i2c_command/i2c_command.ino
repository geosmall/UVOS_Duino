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

static TwoWire::Config defaultWireCfg = [] {
    TwoWire::Config cfg{};
    // select I2C2 and default pins; set default speed to 400 kHz, Master mode
    cfg.i2c_config.periph         = I2CHandle::Config::Peripheral::I2C_2;
    cfg.i2c_config.pin_config.scl = Pin(PORTB, 10);
    cfg.i2c_config.pin_config.sda = Pin(PORTB, 11);
    cfg.i2c_config.speed          = I2CHandle::Config::Speed::I2C_400KHZ;
    cfg.i2c_config.mode           = I2CHandle::Config::Mode::I2C_MASTER;
    return cfg;
}();
TwoWire Wire(defaultWireCfg);

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

    // initialise sensor; must supply I2C address (0x76 or 0x77)
    if (!Dps310PressureSensor.begin(Wire, 0x76)) {
        Serial.println("Sensor init FAILED!");
        while (1);
    } else {
        Serial.println("Sensor init complete!");
    }

    //Wait some time, so that the Dps310 start up
    delay(500);

    for (;;) { // Loop forever
        float temperature;
        float pressure;
        uint8_t oversampling = 7;
        int16_t ret;

        Serial.println();

        //lets the Dps310 perform a Single temperature measurement with the last (or standard) configuration
        //The result will be written to the paramerter temperature
        //ret = Dps310PressureSensor.measureTempOnce(temperature);
        //the commented line below does exactly the same as the one above, but you can also config the precision
        //oversampling can be a value from 0 to 7
        //the Dps 310 will perform 2^oversampling internal temperature measurements and combine them to one result with higher precision
        //measurements with higher precision take more time, consult datasheet for more information
        ret = Dps310PressureSensor.measureTempOnce(temperature, oversampling);

        if (ret != 0) {
            //Something went wrong.
            //Look at the library code for more information about return codes
            Serial.print("FAIL! ret = ");
            Serial.println(ret);
        } else {
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.println(" degrees of Celsius");
        }

        //Pressure measurement behaves like temperature measurement
        //ret = Dps310PressureSensor.measurePressureOnce(pressure);
        ret = Dps310PressureSensor.measurePressureOnce(pressure, oversampling);
        if (ret != 0) {
            //Something went wrong.
            //Look at the library code for more information about return codes
            Serial.print("FAIL! ret = ");
            Serial.println(ret);
        } else {
            Serial.print("Pressure: ");
            Serial.print(pressure);
            Serial.println(" Pascal");
        }

        //Wait some time
        delay(500);
    }

    return 0;
}
