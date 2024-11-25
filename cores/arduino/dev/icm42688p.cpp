#include "icm42688p.h"
#include "registers.h"
#include "sys/system.h"
#include <cstring>

using namespace ICM42688reg;

namespace uvos
{

/* ICM42688 object, input the SPI bus and chip select pin */
ICM42688::ICM42688(SpiHandle bus, uint32_t SPI_HS_CLK) {
  _spi = bus; // SPI bus
  _useSPI = true; // set to use SPI
  SPI_HS_CLOCK = SPI_HS_CLK;
}

/* starts communication with the ICM42688 */
int ICM42688::begin() {
  if( _useSPI ) { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = false;

    /* Wait some time for ICM to be properly supplied */
    System::Delay(5);

    // If the NSS pin is configured as SOFT, initialize as SW NSS pin
    nss_pin_is_SOFT = (_spi.GetConfig().nss == SpiHandle::Config::NSS::SOFT);
    if (nss_pin_is_SOFT)
    {
      initializeSoftNSSPin();
    }
  }

  // reset the ICM42688
  reset();

  // check the WHO AM I byte
  uint8_t id = whoAmI();
  if(id != WHO_AM_I) {
    return -3;
  }

  // turn on accel and gyro in Low Noise (LN) Mode
  if(writeRegister(UB0_REG_PWR_MGMT0, 0x0F) < 0) {
    return -4;
  }

  // 16G is default -- do this to set up accel resolution scaling
  int ret = setAccelFS(gpm16);
  if (ret < 0) return ret;

  // 2000DPS is default -- do this to set up gyro resolution scaling
  ret = setGyroFS(dps2000);
  if (ret < 0) return ret;

  // // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
  // if (setFilters(false, false) < 0) {
  //   return -7;
  // }

  // estimate gyro bias
  if (calibrateGyro() < 0) {
    return -8;
  }
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int ICM42688::setAccelFS(AccelFS fssel) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  _accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
  _accelFS = fssel;

  return 1;
}

/* sets the gyro full scale range to values other than default */
int ICM42688::setGyroFS(GyroFS fssel) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  _gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
  _gyroFS = fssel;

  return 1;
}

int ICM42688::setAccelODR(ODR odr) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  return 1;
}

int ICM42688::setGyroODR(ODR odr) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  return 1;
}

int ICM42688::setFilters(bool gyroFilters, bool accFilters) {
  if (setBank(1) < 0) return -1;

  if (gyroFilters == true) {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_ENABLE) < 0) {
      return -2;
    }
  }
  else {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0) {
      return -3;
    }
  }
  
  if (setBank(2) < 0) return -4;

  if (accFilters == true) {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0) {
      return -5;
    }
  }
  else {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0) {
      return -6;
    }
  }
  if (setBank(0) < 0) return -7;
  return 1;
}

int ICM42688::enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  // push-pull, pulsed, active HIGH interrupts
  if (writeRegister(UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0) return -1;

  // need to clear bit 4 to allow proper INT1 and INT2 operation
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -2;
  reg &= ~0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -3;

  // route UI data ready interrupt to INT1
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x18) < 0) return -4;

  return 1;
}

int ICM42688::disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  // set pin 4 to return to reset value
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -1;
  reg |= 0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -2;

  // return reg to reset value
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x10) < 0) return -3;

  return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int ICM42688::getAGT() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM42688
  if (readRegisters(UB0_REG_TEMP_DATA1, 14, _rxBuffer) < 0) return -1;

  // combine bytes into 16 bit values
  int16_t rawMeas[7]; // temp, accel xyz, gyro xyz
  for (size_t i=0; i<7; i++) {
    rawMeas[i] = ((int16_t)_rxBuffer[i*2] << 8) | _rxBuffer[i*2+1];
  }

  _t = (static_cast<float>(rawMeas[0]) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

  _acc[0] = ((rawMeas[1] * _accelScale) - _accB[0]) * _accS[0];
  _acc[1] = ((rawMeas[2] * _accelScale) - _accB[1]) * _accS[1];
  _acc[2] = ((rawMeas[3] * _accelScale) - _accB[2]) * _accS[2];

  _gyr[0] = (rawMeas[4] * _gyroScale) - _gyrB[0];
  _gyr[1] = (rawMeas[5] * _gyroScale) - _gyrB[1];
  _gyr[2] = (rawMeas[6] * _gyroScale) - _gyrB[2];

  return 1;
}

/* configures and enables the FIFO buffer  */
int ICM42688_FIFO::enableFifo(bool accel,bool gyro,bool temp) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(temp*FIFO_TEMP_EN)) < 0) {
    return -2;
  }
  _enFifoAccel = accel;
  _enFifoGyro = gyro;
  _enFifoTemp = temp;
  _fifoFrameSize = accel*6 + gyro*6 + temp*2;
  return 1;
}

/* reads data from the ICM42688 FIFO and stores in buffer */
int ICM42688_FIFO::readFifo() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // get the fifo size
  readRegisters(UB0_REG_FIFO_COUNTH, 2, _rxBuffer);
  _fifoSize = (((uint16_t) (_rxBuffer[0]&0x0F)) <<8) + (((uint16_t) _rxBuffer[1]));
  // read and parse the buffer
  for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
    // grab the data from the ICM42688
    if (readRegisters(UB0_REG_FIFO_DATA, _fifoFrameSize, _rxBuffer) < 0) {
      return -1;
    }
    if (_enFifoAccel) {
      // combine into 16 bit values
      int16_t rawMeas[3];
      rawMeas[0] = (((int16_t)_rxBuffer[0]) << 8) | _rxBuffer[1];
      rawMeas[1] = (((int16_t)_rxBuffer[2]) << 8) | _rxBuffer[3];
      rawMeas[2] = (((int16_t)_rxBuffer[4]) << 8) | _rxBuffer[5];
      // transform and convert to float values
      _axFifo[i] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
      _ayFifo[i] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
      _azFifo[i] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];
      _aSize = _fifoSize / _fifoFrameSize;
    }
    if (_enFifoTemp) {
      // combine into 16 bit values
      int16_t rawMeas = (((int16_t)_rxBuffer[0 + _enFifoAccel*6]) << 8) | _rxBuffer[1 + _enFifoAccel*6];
      // transform and convert to float values
      _tFifo[i] = (static_cast<float>(rawMeas) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;
      _tSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoGyro) {
      // combine into 16 bit values
      int16_t rawMeas[3];
      rawMeas[0] = (((int16_t)_rxBuffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _rxBuffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
      rawMeas[1] = (((int16_t)_rxBuffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _rxBuffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
      rawMeas[2] = (((int16_t)_rxBuffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _rxBuffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
      // transform and convert to float values
      _gxFifo[i] = (rawMeas[0] * _gyroScale) - _gyrB[0];
      _gyFifo[i] = (rawMeas[1] * _gyroScale) - _gyrB[1];
      _gzFifo[i] = (rawMeas[2] * _gyroScale) - _gyrB[2];
      _gSize = _fifoSize/_fifoFrameSize;
    }
  }
  return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void ICM42688_FIFO::getFifoAccelX_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_axFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void ICM42688_FIFO::getFifoAccelY_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_ayFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void ICM42688_FIFO::getFifoAccelZ_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_azFifo,_aSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, dps */
void ICM42688_FIFO::getFifoGyroX(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gxFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, dps */
void ICM42688_FIFO::getFifoGyroY(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gyFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, dps */
void ICM42688_FIFO::getFifoGyroZ(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gzFifo,_gSize*sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void ICM42688_FIFO::getFifoTemperature_C(size_t *size,float* data) {
  *size = _tSize;
  memcpy(data,_tFifo,_tSize*sizeof(float));
}

/* estimates the gyro biases */
int ICM42688::calibrateGyro() {
  // set at a lower range (more resolution) since IMU not moving
  const GyroFS current_fssel = _gyroFS;
  if (setGyroFS(dps250) < 0) return -1;

  // take samples and find bias
  _gyroBD[0] = 0;
  _gyroBD[1] = 0;
  _gyroBD[2] = 0;
  for (size_t i=0; i < NUM_CALIB_SAMPLES; i++) {
    getAGT();
    _gyroBD[0] += (gyrX() + _gyrB[0]) / NUM_CALIB_SAMPLES;
    _gyroBD[1] += (gyrY() + _gyrB[1]) / NUM_CALIB_SAMPLES;
    _gyroBD[2] += (gyrZ() + _gyrB[2]) / NUM_CALIB_SAMPLES;
    System::Delay(1);
  }
  _gyrB[0] = _gyroBD[0];
  _gyrB[1] = _gyroBD[1];
  _gyrB[2] = _gyroBD[2];

  // recover the full scale setting
  if (setGyroFS(current_fssel) < 0) return -4;
  return 1;
}

/* returns the gyro bias in the X direction, dps */
float ICM42688::getGyroBiasX() {
  return _gyrB[0];
}

/* returns the gyro bias in the Y direction, dps */
float ICM42688::getGyroBiasY() {
  return _gyrB[1];
}

/* returns the gyro bias in the Z direction, dps */
float ICM42688::getGyroBiasZ() {
  return _gyrB[2];
}

/* sets the gyro bias in the X direction to bias, dps */
void ICM42688::setGyroBiasX(float bias) {
  _gyrB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, dps */
void ICM42688::setGyroBiasY(float bias) {
  _gyrB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, dps */
void ICM42688::setGyroBiasZ(float bias) {
  _gyrB[2] = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int ICM42688::calibrateAccel() {
  // set at a lower range (more resolution) since IMU not moving
  const AccelFS current_fssel = _accelFS;
  if (setAccelFS(gpm2) < 0) return -1;

  // take samples and find min / max
  _accBD[0] = 0;
  _accBD[1] = 0;
  _accBD[2] = 0;
  for (size_t i=0; i < NUM_CALIB_SAMPLES; i++) {
    getAGT();
    _accBD[0] += (accX()/_accS[0] + _accB[0]) / NUM_CALIB_SAMPLES;
    _accBD[1] += (accY()/_accS[1] + _accB[1]) / NUM_CALIB_SAMPLES;
    _accBD[2] += (accZ()/_accS[2] + _accB[2]) / NUM_CALIB_SAMPLES;
    System::Delay(1);
  }
  if (_accBD[0] > 0.9f) {
    _accMax[0] = _accBD[0];
  }
  if (_accBD[1] > 0.9f) {
    _accMax[1] = _accBD[1];
  }
  if (_accBD[2] > 0.9f) {
    _accMax[2] = _accBD[2];
  }
  if (_accBD[0] < -0.9f) {
    _accMin[0] = _accBD[0];
  }
  if (_accBD[1] < -0.9f) {
    _accMin[1] = _accBD[1];
  }
  if (_accBD[2] < -0.9f) {
    _accMin[2] = _accBD[2];
  }

  // find bias and scale factor
  if ((abs(_accMin[0]) > 0.9f) && (abs(_accMax[0]) > 0.9f)) {
    _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
    _accS[0] = 1/((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
  }
  if ((abs(_accMin[1]) > 0.9f) && (abs(_accMax[1]) > 0.9f)) {
    _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
    _accS[1] = 1/((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
  }
  if ((abs(_accMin[2]) > 0.9f) && (abs(_accMax[2]) > 0.9f)) {
    _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
    _accS[2] = 1/((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
  }

  // recover the full scale setting
  if (setAccelFS(current_fssel) < 0) return -4;
  return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float ICM42688::getAccelBiasX_mss() {
  return _accB[0];
}

/* returns the accelerometer scale factor in the X direction */
float ICM42688::getAccelScaleFactorX() {
  return _accS[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float ICM42688::getAccelBiasY_mss() {
  return _accB[1];
}

/* returns the accelerometer scale factor in the Y direction */
float ICM42688::getAccelScaleFactorY() {
  return _accS[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float ICM42688::getAccelBiasZ_mss() {
  return _accB[2];
}

/* returns the accelerometer scale factor in the Z direction */
float ICM42688::getAccelScaleFactorZ() {
  return _accS[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void ICM42688::setAccelCalX(float bias,float scaleFactor) {
  _accB[0] = bias;
  _accS[0] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void ICM42688::setAccelCalY(float bias,float scaleFactor) {
  _accB[1] = bias;
  _accS[1] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void ICM42688::setAccelCalZ(float bias,float scaleFactor) {
  _accB[2] = bias;
  _accS[2] = scaleFactor;
}

int ICM42688::setBank(uint8_t bank) {
  // if we are already on this bank, bail
  if (_bank == bank) return 1;

  _bank = bank;

  return writeRegister(REG_BANK_SEL, bank);
}

void ICM42688::reset() {
  setBank(0);

  writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

  // wait for ICM42688 to come back up
  System::Delay(1);
}

/* gets the ICM42688 WHO_AM_I register value */
uint8_t ICM42688::whoAmI() {
  setBank(0);

  // read the WHO AM I register
  if (readRegisters(UB0_REG_WHO_AM_I, 1, _rxBuffer) < 0) {
    return -1;
  }
  // return the register value
  return _rxBuffer[1];
}

int ICM42688::writeRegister(uint8_t addr, uint8_t data)
{
  if (nss_pin_is_SOFT) uvs_gpio_write(&_csPin, 0);
  _txBuffer[0] = addr;
  _txBuffer[1] = data;
  _spi.BlockingTransmitAndReceive(_txBuffer, _rxBuffer, 2);
  if (nss_pin_is_SOFT) uvs_gpio_write(&_csPin, 1);

  System::Delay(10);

  /* Read back the register, addr in rxBuffer_[0], data in rxBuffer_[1] */
  readRegisters(addr, 1, _rxBuffer);
  /* Check the read back register against the written register */
  if (_rxBuffer[1] == data) {
    return 1;
  } else {
    return -1;
  }
}

/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
int ICM42688::readRegisters(uint8_t addr, uint8_t count, uint8_t* dest) {
  // Verify requested size fits in buffer, must be
  // ICM42688_BUFFER_SIZE -1 to cover address byte
  if (count > (ICM42688_BUFFER_SIZE-1)) return -1;

  if (nss_pin_is_SOFT) uvs_gpio_write(&_csPin, 0);
  _txBuffer[0] = (addr | 0x80);
  _txBuffer[1] = 0; // in case writeRegister() has modified
  _spi.BlockingTransmitAndReceive(_txBuffer, dest, count+1);
  if (nss_pin_is_SOFT) uvs_gpio_write(&_csPin, 1);

  return 1;
}

void ICM42688::initializeSoftNSSPin()
{
  _csPin.pin = _spi.GetConfig().pin_config.nss;
  _csPin.mode = UVS_GPIO_MODE_OUTPUT_PP;
  _csPin.pull = UVS_GPIO_NOPULL;
  uvs_gpio_write(&_csPin, 1);
  uvs_gpio_init(&_csPin);
}

#if 0 // gls

ICM42688::Result ICM42688::readRegisters(uint8_t addr, uint8_t count, uint8_t* dest)
{
    // Verify requested size fits in buffer, must be
    // ICM42688_BUFFER_SIZE -1 to cover address byte
    if (count > (ICM42688_BUFFER_SIZE-1)) return Result::ERR;

    if (nss_pin_is_SOFT) uvs_gpio_write(&nss_pin_, 0);
    txBuffer_[0] = (addr | 0x80);
    txBuffer_[1] = 0; // in case writeRegister() has modified
    spi_.BlockingTransmitAndReceive(txBuffer_, dest, count+1);
    if (nss_pin_is_SOFT) uvs_gpio_write(&nss_pin_, 1);

    return Result::OK;
}

uint8_t ICM42688::readRegister(uint8_t addr)
{
    if (nss_pin_is_SOFT) uvs_gpio_write(&nss_pin_, 0);
    txBuffer_[0] = (addr | 0x80);
    txBuffer_[1] = 0; // In case writeRegister() has modified
    spi_.BlockingTransmitAndReceive(txBuffer_, rxBuffer_, 2);
    if (nss_pin_is_SOFT) uvs_gpio_write(&nss_pin_, 1);

    return rxBuffer_[1];
}

ICM42688::Result ICM42688::writeRegister(uint8_t addr, uint8_t data)
{
    if (nss_pin_is_SOFT) uvs_gpio_write(&nss_pin_, 0);
    txBuffer_[0] = addr;
    txBuffer_[1] = data;
    spi_.BlockingTransmitAndReceive(txBuffer_, rxBuffer_, 2);
    if (nss_pin_is_SOFT) uvs_gpio_write(&nss_pin_, 1);

    System::Delay(10);

    /* Read back the register, addr in rxBuffer_[0], data in rxBuffer_[1] */
    readRegisters(addr, 1, rxBuffer_);
    /* Check the read back register against the written register */
    return (rxBuffer_[1] == data) ? Result::OK : Result::ERR;
}

ICM42688::Result ICM42688::writeRegisterMask(uint8_t addr, uint8_t data, uint8_t mask)
{
    // Read current register value and check for error
    if (readRegisters(addr, 1, rxBuffer_) != Result::OK) return Result::ERR;

    // Addr in rxBuffer_[0], data in rxBuffer_[1]
    uint8_t reg = rxBuffer_[1];

    // Apply mask to data and write to register
    return writeRegister(addr, (reg & ~mask) | (data & mask));
}

ICM42688::Result ICM42688::setBank(uint8_t bank)
{
    // If we are already on this bank, return immediately.
    if (_bank == bank)
    {
        return Result::ERR;
    }

    _bank = bank;
    return writeRegister(REG_BANK_SEL, bank);
}

void ICM42688::reset()
{
    setBank(0);

    writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

    // Wait for ICM42688 to come back up
    System::Delay(1);
}

int16_t ICM42688::whoAmI()
{
    setBank(0);

    // Read the WHO AM I register
    if (readRegisters(UB0_REG_WHO_AM_I, 1, rxBuffer_) != Result::OK)
    {
        return -1;
    }
    // Return the WHO_AM_I register value.
    // addr in rxBuffer_[0], data in rxBuffer_[1]
    return (rxBuffer_[1] & 0xFF);
}

void ICM42688::initializeSoftNSSPin()
{
    nss_pin_.pin = spi_.GetConfig().pin_config.nss;
    nss_pin_.mode = UVS_GPIO_MODE_OUTPUT_PP;
    nss_pin_.pull = UVS_GPIO_NOPULL;
    uvs_gpio_init(&nss_pin_);
}

#endif // gls

} // namespace uvos
