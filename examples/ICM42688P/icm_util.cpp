#include "icm_util.h"
#include "stm32h7xx_ll_spi.h"

#define ICM42605_RA_PWR_MGMT0                       0x4E

#define ICM42605_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM42605_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM42605_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM42605_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

#define ICM42605_RA_GYRO_CONFIG0                    0x4F
#define ICM42605_RA_ACCEL_CONFIG0                   0x50

#define ICM42605_RA_GYRO_ACCEL_CONFIG0              0x52

#define ICM42605_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4)
#define ICM42605_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)

#define ICM42605_RA_GYRO_DATA_X1                    0x25
#define ICM42605_RA_ACCEL_DATA_X1                   0x1F

#define ICM42605_RA_INT_CONFIG                      0x14
#define ICM42605_INT1_MODE_PULSED                   (0 << 2)
#define ICM42605_INT1_MODE_LATCHED                  (1 << 2)
#define ICM42605_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM42605_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM42605_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM42605_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM42605_RA_INT_CONFIG0                     0x63
#define ICM42605_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM42605_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM42605_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM42605_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM42605_RA_INT_CONFIG1                     0x64
#define ICM42605_INT_ASYNC_RESET_BIT                4
#define ICM42605_INT_TDEASSERT_DISABLE_BIT          5
#define ICM42605_INT_TDEASSERT_ENABLED              (0 << ICM42605_INT_TDEASSERT_DISABLE_BIT)
#define ICM42605_INT_TDEASSERT_DISABLED             (1 << ICM42605_INT_TDEASSERT_DISABLE_BIT)
#define ICM42605_INT_TPULSE_DURATION_BIT            6
#define ICM42605_INT_TPULSE_DURATION_100            (0 << ICM42605_INT_TPULSE_DURATION_BIT)
#define ICM42605_INT_TPULSE_DURATION_8              (1 << ICM42605_INT_TPULSE_DURATION_BIT)

#define ICM42605_RA_INT_SOURCE0                     0x65
#define ICM42605_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM42605_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

#define ICM42605_INTF_CONFIG1                       0x4D
#define ICM42605_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM42605_INTF_CONFIG1_AFSR_DISABLE          0x40

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2

#define MPU_RA_WHO_AM_I             0x75

#define ICM42605_WHO_AM_I_CONST    (0x42)
#define ICM42688P_WHO_AM_I_CONST   (0x47)

static bool is42688P = false;

typedef struct aafConfig_s {
    uint16_t freq;
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshift;
} aafConfig_t;

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42688[] = {  // see table in section 5.3 https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf
    // freq, delt, deltSqr, bitshift
    { 42,  1,    1, 15 },
    { 84,  2,    4, 13 },
    {126,  3,    9, 12 },
    {170,  4,   16, 11 },
    {213,  5,   25, 10 },
    {258,  6,   36, 10 },
    {303,  7,   49,  9 },
    {536, 12,  144,  8 },
    {997, 21,  440,  6 },
    {1962, 37, 1376,  4 },
    { 0, 0, 0, 0}, // 42HZ
};

// Possible gyro Anti-Alias Filter (AAF) cutoffs for ICM-42688P
static aafConfig_t aafLUT42605[] = {  // see table in section 5.3 https://invensense.tdk.com/wp-content/uploads/2022/09/DS-000292-ICM-42605-v1.7.pdf
    // freq, delt, deltSqr, bitshift
    {  10,  1,    1, 15 },
    {  21,  2,    4, 13 },
    {  32,  3,    9, 12 },
    {  42,  4,   16, 11 },
    {  99,  9,   81,  9 },
    { 171, 15,  224,  7 },
    { 184, 16,  256,  7 },
    { 196, 17,  288,  7 },
    { 249, 21,  440,  6 },
    { 524, 39, 1536,  4 },
    { 995, 63, 3968,  3 },
    {   0,  0,    0,  0 }
};

static const aafConfig_t* getGyroAafConfig(bool is42688, const uint16_t desiredLpf);

static const gyroFilterAndRateConfig_t icm42605GyroConfigs[] = {
    /*                            DLPF  ODR */
    { GYRO_LPF_256HZ,   8000,   { 6,    3  } }, /* 400 Hz LPF */
    { GYRO_LPF_256HZ,   4000,   { 5,    4  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,   2000,   { 3,    5  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,   1000,   { 1,    6  } }, /* 250 Hz LPF */
    { GYRO_LPF_256HZ,    500,   { 0,    15 } }, /* 250 Hz LPF */

    { GYRO_LPF_188HZ,   1000,   { 3,   6  } },  /* 125 HZ */
    { GYRO_LPF_188HZ,    500,   { 1,   15 } },  /* 125 HZ */

    { GYRO_LPF_98HZ,    1000,   { 4,    6  } }, /* 100 HZ*/
    { GYRO_LPF_98HZ,     500,   { 2,    15 } }, /* 100 HZ*/

    { GYRO_LPF_42HZ,    1000,   { 6,    6  } }, /* 50 HZ */
    { GYRO_LPF_42HZ,     500,   { 4,    15 } },

    { GYRO_LPF_20HZ,    1000,   { 7,    6  } }, /* 25 HZ */
    { GYRO_LPF_20HZ,     500,   { 6,    15 } },

    { GYRO_LPF_10HZ,    1000,   { 7,    6  } }, /* 25 HZ */
    { GYRO_LPF_10HZ,     500,   { 7,    15 } }  /* 12.5 HZ */
};

// Create local INav busDevice object
static busDevice_t spi_dev_;

// Use the uvos namespace to prevent having to type
// uvos:: before all libuvos functions
using namespace uvos;

void spiIcmBusInit(const busDevice_t* dev)
{
    // Copy 'dev' to local static 'spi_dev_' for later use
    spi_dev_ = *dev;
}

void spiChipSelectSetupDelay(void)
{
    // CS->CLK delay, MPU6000 - 8ns
    // CS->CLK delay, ICM42688P - 39ns
    System::DelayNs(39);
}

void spiChipSelectHoldTime(void)
{
    // CLK->CS delay, MPU6000 - 500ns
    // CS->CLK delay, ICM42688P - 18ns
    System::DelayNs(18);
}

void spiBusSelectDevice(const busDevice_t* dev)
{
    const_cast<uvos::GPIO&>(dev->spi.csnPin).Write(GPIO_PIN_RESET);
    spiChipSelectSetupDelay();
}

void spiBusDeselectDevice(const busDevice_t* dev)
{
    spiChipSelectHoldTime();
    const_cast<uvos::GPIO&>(dev->spi.csnPin).Write(GPIO_PIN_SET);
}

bool spiTransfer(SPI_TypeDef *instance, uint8_t *rxData, const uint8_t *txData, int len)
{
    LL_SPI_SetTransferSize(instance, len);
    LL_SPI_Enable(instance);
    LL_SPI_StartMasterTransfer(instance);
    while (len) {
        int spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_TXP(instance)) {
            if ((spiTimeout--) == 0) {
                // spiTimeoutUserCallback(instance); gls
                return false;
            }
        }
        uint8_t b = txData ? *(txData++) : 0xFF;
        LL_SPI_TransmitData8(instance, b);

        spiTimeout = 1000;
        while (!LL_SPI_IsActiveFlag_RXP(instance)) {
            if ((spiTimeout--) == 0) {
                // spiTimeoutUserCallback(instance); gls
                return false;
            }
        }
        b = LL_SPI_ReceiveData8(instance);
        if (rxData) {
            *(rxData++) = b;
        }
        --len;
    }
    while (!LL_SPI_IsActiveFlag_EOT(instance));
    // Add a delay before disabling SPI otherwise last-bit/last-clock may be truncated
    // See https://github.com/stm32duino/Arduino_Core_STM32/issues/1294
    // Computed delay is half SPI clock
    System::DelayUs(spi_dev_.spi.disable_delay);

    LL_SPI_ClearFlag_TXTF(instance);
    LL_SPI_Disable(instance);

    return true;
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t txByte)
{
    uint8_t value = 0xFF;
    if (!spiTransfer(instance, &value, &txByte, 1)) {
        return 0xFF;
    }
    return value;
}

void busSetSpeed(const busDevice_t* dev, busSpeed_e speed)
{

}

bool spiBusWriteRegister(const busDevice_t* dev, uint8_t reg, uint8_t data)
{
    SpiHandle::Config cfg = dev->spi.spiBus.GetConfig();
    SPI_TypeDef* instance = SpiHandle::PeripheralToHAL(cfg.periph);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusSelectDevice(dev);
    }

    spiTransferByte(instance, reg);
    spiTransferByte(instance, data);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusDeselectDevice(dev);
    }

    return true;
}

bool spiBusReadRegister(const busDevice_t* dev, uint8_t reg, uint8_t* data)
{
    SpiHandle::Config cfg = dev->spi.spiBus.GetConfig();
    SPI_TypeDef* instance = SpiHandle::PeripheralToHAL(cfg.periph);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusSelectDevice(dev);
    }

    spiTransferByte(instance, reg);
    spiTransfer(instance, data, NULL, 1);

    if (!(dev->flags & DEVFLAGS_USE_MANUAL_DEVICE_SELECT)) {
        spiBusDeselectDevice(dev);
    }

    return true;
}

bool busWrite(const busDevice_t* dev, uint8_t reg, uint8_t data)
{
    if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
        return spiBusWriteRegister(dev, reg, data);
    } else {
        return spiBusWriteRegister(dev, reg & 0x7F, data);
    }
}

bool busRead(const busDevice_t* dev, uint8_t reg, uint8_t* data)
{
    if (dev->flags & DEVFLAGS_USE_RAW_REGISTERS) {
        return spiBusReadRegister(dev, reg, data);
    } else {
        return spiBusReadRegister(dev, reg | 0x80, data);
    }
}

static void setUserBank(const busDevice_t *dev, const uint8_t user_bank)
{
    busWrite(dev, ICM426XX_RA_REG_BANK_SEL, user_bank & 7);
}

bool icm42605DeviceDetect(const busDevice_t* dev)
{
    uint8_t tmp;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    busWrite(dev, ICM42605_RA_PWR_MGMT0, 0x00);

    do {
        System::Delay(150);

        busRead(dev, MPU_RA_WHO_AM_I, &tmp);

        switch (tmp) {
        /* ICM42605 and ICM42688P share the register structure*/
        case ICM42605_WHO_AM_I_CONST:
            is42688P = false;
            return true;
        case ICM42688P_WHO_AM_I_CONST:
            is42688P = true;
            return true;

        default:
            // Retry detection
            break;
        }
    } while (attemptsRemaining--);

    return false;
}

const gyroFilterAndRateConfig_t* chooseGyroConfig(uint8_t desiredLpf, uint16_t desiredRateHz, const gyroFilterAndRateConfig_t* configs, int count)
{
    int i;
    int8_t selectedLpf = configs[0].gyroLpf;
    const gyroFilterAndRateConfig_t* candidate = &configs[0];

    // Choose closest supported LPF value
    for (i = 1; i < count; i++) {
        if (ABS(desiredLpf - configs[i].gyroLpf) < ABS(desiredLpf - selectedLpf)) {
            selectedLpf = configs[i].gyroLpf;
            candidate = &configs[i];
        }
    }

    // Now find the closest update rate
    for (i = 0; i < count; i++) {
        if ((configs[i].gyroLpf == selectedLpf) && (ABS(desiredRateHz - candidate->gyroRateHz) > ABS(desiredRateHz - configs[i].gyroRateHz))) {
            candidate = &configs[i];
        }
    }

    // LOG_VERBOSE(GYRO, "GYRO CONFIG { %d, %d } -> { %d, %d}; regs 0x%02X, 0x%02X",
    //             desiredLpf, desiredRateHz,
    //             candidate->gyroLpf, candidate->gyroRateHz,
    //             candidate->gyroConfigValues[0], candidate->gyroConfigValues[1]);

    return candidate;
}

static uint16_t getAafFreq(const uint8_t gyroLpf)
{
    switch (gyroLpf) {
    default:
    case GYRO_LPF_256HZ:
        return 256;
    case GYRO_LPF_188HZ:
        return 188;
    case GYRO_LPF_98HZ:
        return 98;
    case GYRO_LPF_42HZ:
        return 42;
    case GYRO_LPF_20HZ:
        return 20;
    case GYRO_LPF_10HZ:
        return 10;
    case GYRO_LPF_5HZ:
        return 5;
    case GYRO_LPF_NONE:
        return 0;
    }
}

static const aafConfig_t* getGyroAafConfig(bool is42688, const uint16_t desiredLpf)
{
    uint16_t desiredFreq = getAafFreq(desiredLpf);
    const aafConfig_t* aafConfigs = NULL;
    if (is42688) {
        aafConfigs = aafLUT42688;
    } else {
        aafConfigs = aafLUT42605;
    }
    int i;
    int8_t selectedFreq = aafConfigs[0].freq;
    const aafConfig_t* candidate = &aafConfigs[0];

    // Choose closest supported LPF value
    for (i = 1; aafConfigs[i].freq != 0; i++) {
        if (ABS(desiredFreq - aafConfigs[i].freq) < ABS(desiredFreq - selectedFreq)) {
            selectedFreq = aafConfigs[i].freq;
            candidate = &aafConfigs[i];
        }
    }

    // LOG_VERBOSE(GYRO, "ICM426%s AAF CONFIG { %d, %d } -> { %d }; delt: %d deltSqr: %d, shift: %d",
    //     (is42688P ? "88" : "05"),
    //             desiredLpf, desiredFreq,
    //             candidate->freq,
    //             candidate->delt, candidate->deltSqr, candidate->bitshift);

    return candidate;
}

void icm42605AccAndGyroInit(gyroDev_t *gyro)
{
    busDevice_t* dev = gyro->busDev;
    const gyroFilterAndRateConfig_t * config = chooseGyroConfig(gyro->lpf, 1000000 / gyro->requestedSampleIntervalUs,
                                                                &icm42605GyroConfigs[0], ARRAYLEN(icm42605GyroConfigs));
    gyro->sampleRateIntervalUs = 1000000 / config->gyroRateHz;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    setUserBank(dev, ICM426XX_BANK_SELECT0);
    busWrite(dev, ICM42605_RA_PWR_MGMT0, ICM42605_PWR_MGMT0_TEMP_DISABLE_OFF | ICM42605_PWR_MGMT0_ACCEL_MODE_LN | ICM42605_PWR_MGMT0_GYRO_MODE_LN);
    System::Delay(15);

    /* ODR and dynamic range */
    busWrite(dev, ICM42605_RA_GYRO_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 2000 deg/s */
    System::Delay(15);

    busWrite(dev, ICM42605_RA_ACCEL_CONFIG0, (0x00) << 5 | (config->gyroConfigValues[1] & 0x0F));    /* 16 G deg/s */
    System::Delay(15);

    /* LPF bandwidth */
    // low latency, same as BF
    busWrite(dev, ICM42605_RA_GYRO_ACCEL_CONFIG0, ICM42605_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM42605_GYRO_UI_FILT_BW_LOW_LATENCY);
    System::Delay(15);

    if (gyro->lpf != GYRO_LPF_NONE) {
        // Configure gyro Anti-Alias Filter (see section 5.3 "ANTI-ALIAS FILTER")
        const aafConfig_t *aafConfig = getGyroAafConfig(is42688P, gyro->lpf);
    
        setUserBank(dev, ICM426XX_BANK_SELECT1);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC3, aafConfig->delt);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC4, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM426XX_RA_GYRO_CONFIG_STATIC5, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));

        aafConfig = getGyroAafConfig(is42688P, 256);  // This was hard coded on BF
        setUserBank(dev, ICM426XX_BANK_SELECT2);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC2, aafConfig->delt << 1);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC3, aafConfig->deltSqr & 0xFF);
        busWrite(dev, ICM426XX_RA_ACCEL_CONFIG_STATIC4, (aafConfig->deltSqr >> 8) | (aafConfig->bitshift << 4));
    }

    setUserBank(dev, ICM426XX_BANK_SELECT0);
    busWrite(dev, ICM42605_RA_INT_CONFIG, ICM42605_INT1_MODE_PULSED | ICM42605_INT1_DRIVE_CIRCUIT_PP | ICM42605_INT1_POLARITY_ACTIVE_HIGH);
    System::Delay(15);

    busWrite(dev, ICM42605_RA_INT_CONFIG0, ICM42605_UI_DRDY_INT_CLEAR_ON_SBR);
    System::Delay(100);

    uint8_t intConfig1Value;

    busWrite(dev, ICM42605_RA_INT_SOURCE0, ICM42605_UI_DRDY_INT1_EN_ENABLED);

    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
    busRead(dev, ICM42605_RA_INT_CONFIG1, &intConfig1Value);

    intConfig1Value &= ~(1 << ICM42605_INT_ASYNC_RESET_BIT);
    intConfig1Value |= (ICM42605_INT_TPULSE_DURATION_8 | ICM42605_INT_TDEASSERT_DISABLED);

    busWrite(dev, ICM42605_RA_INT_CONFIG1, intConfig1Value);
    System::Delay(15);

    //Disable AFSR as in BF and Ardupilot
    uint8_t intfConfig1Value;
    busRead(dev, ICM42605_INTF_CONFIG1, &intfConfig1Value);
    intfConfig1Value &= ~ICM42605_INTF_CONFIG1_AFSR_MASK;
    intfConfig1Value |= ICM42605_INTF_CONFIG1_AFSR_DISABLE;
    busWrite(dev, ICM42605_INTF_CONFIG1, intfConfig1Value);

    System::Delay(15);

    busSetSpeed(dev, BUS_SPEED_FAST);
}