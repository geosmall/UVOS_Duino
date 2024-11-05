## INAV Mapping

| INAV | Pin | Voltage Tolerance | I/O | Timer Channel | Function               |
|------|-----|-------------------|-----|---------------|------------------------|
| S1   | PB0 | 5 V tolerant      | I/O | TIM3_CH3      | Fixed Wing Motor       |
| S2   | PB1 | 3.3 V tolerant    | I/O | TIM3_CH4      |                        |
| S3   | PA0 | 5 V tolerant      | I/O | TIM5_CH1      | Fixed Wing Servo       |
| S4   | PA1 | 5 V tolerant      | I/O | TIM5_CH2      |                        |
| S5   | PA2 | 5 V tolerant      | I/O | TIM5_CH3      |                        |
| S6   | PA3 | 5 V tolerant      | I/O | TIM5_CH4      |                        |
| S7   | PD12| 5 V tolerant      | I/O | TIM4_CH1      |                        |
| S8   | PD13| 5 V tolerant      | I/O | TIM4_CH2      |                        |
| S9   | PD14| 5 V tolerant      | I/O | TIM4_CH3      |                        |
| S10  | PD15| 5 V tolerant      | I/O | TIM4_CH4      |                        |
| S11  | PE5 | 5 V tolerant      | I/O | TIM15_CH1     |                        |
| S12  | PE6 | 5 V tolerant      | I/O | TIM15_CH2     |                        |
| LED  | PA8 | 5 V tolerant      | I/O | TIM1_CH1      | 2812LED                |

## ADC Mapping

| Function | Pad      | Voltage Divider               | Pin | Voltage Range | ADC Channel      | Scale         |
|----------|----------|-------------------------------|-----|---------------|------------------|---------------|
| Vbat     | Vbat pad | 1K:10K divider builtin        | PC0 | 0~36V        | ADC_CHANNEL_1    | 1100          |
| Current  | Curr Pad | N/A                           | PC1 | 0~3.3V       | ADC_CHANNEL_2    | 150           |
| RSSI     | RSSI Pad | N/A                           | PC5 | 0~3.3V       | ADC_CHANNEL_3    | Analog RSSI   |
| AirSpeed | AirS Pad | 20K:20K divider builtin       | PC4 | 0~6.6V       | ADC_CHANNEL_4    | Analog Airspeed |
| VB2      | VB2 Pad  | 1K:20K divider builtin        | PA4 | 0~69V        | ADC_CHANNEL_5    | 2100          |
| CU2      | CU2 Pad  | N/A                           | PA7 | 0~3.3V       | ADC_CHANNEL_6    | Spare         |

## I2C Mapping

| I2C Bus | Pins                  | Voltage Tolerance | Devices                                                                |
|---------|------------------------|-------------------|------------------------------------------------------------------------|
| I2C1    | CL1/DA1 (PB6/PB7)      | 5 V tolerant I/O | Compass (QMC5883 / HMC5883, IST8310 / IST8308, MAG3110 / LIS3MDL), OLED 0.96″ |
| I2C2    | CL2/DA2 on JST-GH-4P (PB10/PB11) | 5 V tolerant I/O | Onboard Barometer DPS310, Digital Airspeed sensor MS4525, Temperature sensor |

## UART Mapping

| UART      | Pins        | Voltage Tolerance | Function        |
|-----------|-------------|-------------------|-----------------|
| USB       | PA11/PA12   | 5 V tolerant I/O  | USB            |
| TX1/RX1   | PA9/PA10    | 5 V tolerant I/O  | USART1 telem2  |
| TX2/RX2   | PD5/PD6     | 5 V tolerant I/O  | USART2 GPS1    |
| TX3/RX3   | PD8/PD9     | 5 V tolerant I/O  | USART3 GPS2    |
| TX4/RX4   | PB9/PB8     | 5 V tolerant I/O  | UART4 USER     |
| TX6/RX6   | PC6/PC7     | 5 V tolerant I/O  | USART6 RC input/Receiver |
| RX7/TX7   | PE7/PE8     | 3.3 V tolerant I/O| UART7 telem1   |
| TX8/RX8   | PE1/PE0     | 5 V tolerant I/O  | UART8 USER     |

## CAN Mapping

| CAN Bus | Pins       | Voltage Tolerance | Function |
|---------|------------|-------------------|----------|
| CAN1    | PD0/PD1    | 5 V tolerant I/O  | CAN Node |
