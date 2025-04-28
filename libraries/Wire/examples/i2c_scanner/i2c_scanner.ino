#include "uvos_brd.h"
#include <Wire.h>

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

// Declare a UVOSboard object called hw
UVOSboard hw;

static constexpr size_t DMA_BUF_SIZE = 256;   // DMA rx buffer size
static uint8_t DMA_BUFFER_MEM_SECTION HardwareSerial_rx_buf[DMA_BUF_SIZE] = {0};

// Build full HardwareSerial::Config with nested lambda
static const HardwareSerial::Config serial_cfg = []{
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

int main(void)
{
  setup();

  for (;;) // Loop forever
  {
    loop();
  }

  return 0;
}

void setup() {

  Serial.begin(9600);
  Wire.begin();
  Serial.println("\nI2C Scanner");
}


void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");

  delay(5000);           // wait 5 seconds for next scan
}
