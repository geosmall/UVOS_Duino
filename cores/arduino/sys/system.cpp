#ifndef UNIT_TEST // for unit tests, a dummy implementation is provided below
#include <stm32h7xx_hal.h>
#include <stm32yyxx.h>
#include "uvos_core.h"
#include "sys/system.h"
#include "sys/dma.h"
#include "per/gpio.h"

// global init functions for peripheral drivers.
extern "C"
{
    extern void SystemClock_Config(uint32_t plln_val, uint32_t FLatency); // in Variant folder 
    extern void uvs_i2c_global_init();
    extern void uvs_spi_global_init();
    extern void uvs_uart_global_init();
}

// boot info struct declared in persistent backup RAM
volatile uvos::System::BootInfo __attribute__((section(".backup_ram")))
uvos::boot_info;

// Jump related stuff
#define u32 uint32_t
#define vu32 volatile uint32_t
#define SET_REG(addr, val)    \
    do                        \
    {                         \
        *(vu32*)(addr) = val; \
    } while(0)

#define GET_REG(addr) (*(vu32*)(addr))
#define RCC_CR RCC
#define RCC_CFGR (RCC + 0x04)
#define RCC_CIR (RCC + 0x08)
#define RCC_AHBENR (RCC + 0x14)
#define RCC_APB2ENR (RCC + 0x18)
#define RCC_APB1ENR (RCC + 0x1C)

#define SCS 0xE000E000
#define STK (SCS + 0x10)
#define STK_CTRL (STK + 0x00)
#define RCC_CR RCC

#define SDRAM_BASE 0xC0000000

#define INTERNAL_FLASH_SIZE 0x20000
#define ITCMRAM_SIZE 0x10000
#define DTCMRAM_SIZE 0x20000
#define SRAM_D1_SIZE 0x80000
#define SRAM_D2_SIZE 0x48000
#define SRAM_D3_SIZE 0x10000
#define SDRAM_SIZE 0x4000000
#define QSPI_SIZE 0x800000

typedef struct
{
    vu32 ISER[2];
    u32  RESERVED0[30];
    vu32 ICER[2];
    u32  RSERVED1[30];
    vu32 ISPR[2];
    u32  RESERVED2[30];
    vu32 ICPR[2];
    u32  RESERVED3[30];
    vu32 IABR[2];
    u32  RESERVED4[62];
    vu32 IPR[15];
} NVIC_TypeDef;

__attribute__((always_inline)) static inline void __JUMPTOQSPI()
{
    __asm("LDR R1, =0xE000ED00;"); // SCB
    __asm("LDR R0, =0x90000000;"); // APP BASE
    __asm("STR R0, [R1, #8]");     // VTOR
    __asm("LDR SP, [R0, #0]");     // SP @ +0
    __asm("LDR R0, [R0, #4]");     // PC @ +4
    __asm("BX R0");
}

typedef void (*EntryPoint)(void);

/** Static variable to hold DWT ticks per microsecond */
static uint32_t usTicks = 0;

// System Level C functions and IRQ Handlers
extern "C"
{
    void SysTick_Handler(void)
    {
        HAL_IncTick();
        HAL_SYSTICK_IRQHandler();
    }

    /** USB IRQ Handlers since they are shared resources for multiple classes */
    extern HCD_HandleTypeDef hhcd_USB_OTG_HS;
    extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

#if 0 // gls
    void OTG_HS_EP1_OUT_IRQHandler(void)
    {
        // if(hhcd_USB_OTG_HS.Instance)
        //     HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
        if(hpcd_USB_OTG_HS.Instance)
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
    }

    void OTG_HS_EP1_IN_IRQHandler(void)
    {
        // if(hhcd_USB_OTG_HS.Instance)
        //     HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
        if(hpcd_USB_OTG_HS.Instance)
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
    }

    void OTG_HS_IRQHandler(void)
    {
        // if(hhcd_USB_OTG_HS.Instance)
        //     HAL_HCD_IRQHandler(&hhcd_USB_OTG_HS);
        if(hpcd_USB_OTG_HS.Instance)
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
    }
#endif // gls

    // TODO: Add some real handling to the HardFaultHandler
    void HardFault_Handler()
    {
        // Grab an instance of the SCB so we can `p/x *scb` from the debugger
        SCB_Type* scb = SCB;
        (void)(scb);

        // Identify hardfault type
        if(SCB->HFSR & SCB_HFSR_FORCED_Msk)
        {
            // Forced hardfault

            // Copy faults for easy debugging
            size_t mmu_fault = (SCB->CFSR >> 0) & 0xFF;
            (void)(mmu_fault);
            size_t bus_fault = (SCB->CFSR >> 8) & 0xFF;
            (void)(bus_fault);
            size_t usage_fault = (SCB->CFSR >> 16) & 0xFFFF;
            (void)(usage_fault);

            // Check for memory manger faults
            if(SCB->CFSR & SCB_CFSR_MMARVALID_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_MLSPERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_MSTKERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_MUNSTKERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_DACCVIOL_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_IACCVIOL_Msk)
                __asm("BKPT #0");

            // Check for bus faults
            if(SCB->CFSR & SCB_CFSR_BFARVALID_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_LSPERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_STKERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_UNSTKERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_PRECISERR_Msk)
                __asm("BKPT #0");
            if(SCB->CFSR & SCB_CFSR_IBUSERR_Msk)
                __asm("BKPT #0");

            // Check for usage faults
            if(SCB->CFSR & SCB_CFSR_DIVBYZERO_Msk)
                __asm("BKPT 0");
            if(SCB->CFSR & SCB_CFSR_UNALIGNED_Msk)
                __asm("BKPT 0");
            if(SCB->CFSR & SCB_CFSR_NOCP_Msk)
                __asm("BKPT 0");
            if(SCB->CFSR & SCB_CFSR_INVPC_Msk)
                __asm("BKPT 0");
            if(SCB->CFSR & SCB_CFSR_INVSTATE_Msk)
                __asm("BKPT 0");
            if(SCB->CFSR & SCB_CFSR_UNDEFINSTR_Msk)
                __asm("BKPT 0");

            __asm("BKPT #0");
        }
        else if(SCB->HFSR & SCB_HFSR_VECTTBL_Msk)
        {
            // Vector table bus fault

            __asm("BKPT #0");
        }

        __asm("BKPT #0");
        while(1)
            ;
    }
}

namespace uvos
{
// Define static tim_
TimerHandle System::tim_;

void System::Init()
{
    System::Config cfg;
    cfg.Defaults();
    Init(cfg);
}

// TODO: DONT FORGET TO IMPLEMENT CLOCK CONFIG
void System::Init(const System::Config& config)
{
    cfg_ = config;
    HAL_Init();
    if(!config.skip_clocks)
    {
        ConfigureClocks();
        ConfigureMpu();
    }
    uvs_dma_init();
    // uvs_i2c_global_init();
    // uvs_spi_global_init();
    // uvs_uart_global_init();

    // Initialize Caches
    if(config.use_dcache)
        SCB_EnableDCache();
    if(config.use_icache)
        SCB_EnableICache();

    // Init DWT timer, precompute delay ticks
    InitDWT();
    usTicks = SystemCoreClock / 1'000'000;

    // Configure and start highspeed timer.
    // TIM 2 counter UP (defaults to fastest tick/longest period).
    TimerHandle::Config timcfg;
    timcfg.periph = TimerHandle::Config::Peripheral::TIM_2;
    timcfg.dir    = TimerHandle::Config::CounterDir::UP;
    tim_.Init(timcfg);
    tim_.Start();
}

void System::DeInit()
{
    uvs_dma_deinit();
    // HAL_MPU_Disable();
    // The I2C global init doesn't actually initialize the periph,
    // so no need to deinitialize
    SCB_DisableDCache();
    SCB_DisableICache();

    tim_.DeInit();
    // HAL_RCC_DeInit();

    // WARNING -- without modifications, this function will
    // cause the device to reset, preventing program loading
    // HAL_DeInit();
}

uint32_t System::InitDWT()
{
    /* Enable use of DWT */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* Unlock */
    AccessDWT(true);

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* Enable  clock cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 3 NO OPERATION instructions */
    __asm volatile(" nop      \n\t"
                   " nop      \n\t"
                   " nop      \n\t");

    /* Check if clock cycle counter has started */
    return (DWT->CYCCNT) ? 0 : 1;
}

void System::AccessDWT(bool enable)
{
    // https://stackoverflow.com/questions/38355831/measuring-clock-cycle-count-on-cortex-m7
#if !defined DWT_LSR_Present_Msk
#define DWT_LSR_Present_Msk ITM_LSR_Present_Msk
#endif
#if !defined DWT_LSR_Access_Msk
#define DWT_LSR_Access_Msk ITM_LSR_Access_Msk
#endif
    uint32_t lsr = DWT->LSR;

    if ((lsr & DWT_LSR_Present_Msk) != 0)
    {
        if (enable)
        {
            if ((lsr & DWT_LSR_Access_Msk) != 0)
            { // locked
                DWT->LAR = 0xC5ACCE55;
            }
        }
        else
        {
            if ((lsr & DWT_LSR_Access_Msk) == 0)
            { // unlocked
                DWT->LAR = 0;
            }
        }
    }
}

void System::JumpToQspi()
{
    __JUMPTOQSPI();
    while(1) {}
}

uint32_t System::GetTickHAL()
{
    return HAL_GetTick();
}

#if 0 // gls

uint32_t System::GetMs()
{
    return tim_.GetMs();
}

uint32_t System::GetUs()
{
    return tim_.GetUs();
}

uint32_t System::GetTick()
{
    return tim_.GetTick();
}

void System::Delay(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

void System::DelayUs(uint32_t delay_us)
{
    tim_.DelayUs(delay_us);
}

void System::DelayTicks(uint32_t delay_ticks)
{
    tim_.DelayTick(delay_ticks);
}

#endif // gls

void System::Delay(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

inline void System::DelayUs(uint32_t delay_us)
{
    uint32_t start  = GetTicksDWT();
    uint32_t ticks = delay_us * usTicks;
    while ((GetTicksDWT() - start) < ticks)
    {
        asm volatile("" ::: "memory"); // Compiler barrier
    }
}

inline void System::DelayNanos(int32_t delay_ns)
{
    const uint32_t start = GetTicksDWT();
    const uint32_t ticks = (delay_ns * usTicks) / 1000;
    while (GetTicksDWT() - start <= ticks)
    {
        asm volatile("" ::: "memory"); // Compiler barrier
    }
}

inline void System::DelayTicks(uint32_t ticks)
{
    if (ticks == 0) return;
    uint32_t start = GetTicksDWT();
    while ((GetTicksDWT() - start) < ticks)
    {
        asm volatile("" ::: "memory"); // Compiler barrier to prevent optimization
    }
}

void System::ResetToBootloader(BootloaderMode mode)
{
    if(mode == BootloaderMode::STM)
    {
        // Initialize Boot Pin
        uvs_gpio_pin bootpin = {UVS_GPIOG, 3};
        uvs_gpio     pin;
        pin.mode = UVS_GPIO_MODE_OUTPUT_PP;
        pin.pin  = bootpin;
        uvs_gpio_init(&pin);

        // Pull Pin HIGH
        uvs_gpio_write(&pin, 1);

        // wait a few ms for cap to charge
        HAL_Delay(10);
    }
    else if(mode == BootloaderMode::UVOS
            || mode == BootloaderMode::UVOS_SKIP_TIMEOUT
            || mode == BootloaderMode::UVOS_INFINITE_TIMEOUT)
    {
        auto region = GetProgramMemoryRegion();
        if(region == MemoryRegion::INTERNAL_FLASH)
            return; // Cannot return to UVOS bootloader if it's not present!

        // Coming from a bootloaded program, the backup SRAM will already
        // be initialized. If the bootloader is <= v5, then it will not
        // be initialized, but a failed write will not cause a fault.
        switch(mode)
        {
            case BootloaderMode::UVOS_SKIP_TIMEOUT:
                boot_info.status = BootInfo::Type::SKIP_TIMEOUT;
                break;
            case BootloaderMode::UVOS_INFINITE_TIMEOUT:
                boot_info.status = BootInfo::Type::INF_TIMEOUT;
                break;
            default:
                // this is technically valid, just means no
                // special behavior applied on boot
                boot_info.status = BootInfo::Type::INVALID;
                break;
        }
    }
    else
    {
        return; // Malformed mode
    }

    // disable interupts
    RCC->CIER = 0x00000000;

    // Software Reset
    HAL_NVIC_SystemReset();
}

void System::InitBackupSram()
{
    PWR->CR1 |= PWR_CR1_DBP;
    while((PWR->CR1 & PWR_CR1_DBP) == RESET)
        ;
    __HAL_RCC_BKPRAM_CLK_ENABLE();
}

System::BootInfo::Version System::GetBootloaderVersion()
{
    auto region = GetProgramMemoryRegion();
    if(region == MemoryRegion::INTERNAL_FLASH)
        return BootInfo::Version::NONE;

    for(int i = 0; i < (int)BootInfo::Version::LAST; i++)
    {
        if(boot_info.version == (BootInfo::Version)i)
            return (BootInfo::Version)i;
    }

    // Otherwise, the version may be higher than this
    // version of the library knows about. In that case,
    // expecting backwards compatibility, we'll say it's
    // at least the latest version.
    return (BootInfo::Version)((int)BootInfo::Version::LAST - 1);
}

void System::ConfigureClocks()
{

#if !defined(HSE_VALUE)
    #error "HSE_VALUE not defined in system.cpp"
#else
    constexpr uint32_t hseValue = HSE_VALUE;
#endif

#if (HSE_VALUE != 8000000) && (HSE_VALUE != 16000000) && (HSE_VALUE != 25000000)
#error "Unsupported HSE_VALUE"
#endif

    // RCC_OscInitTypeDef RCC_OscInitStruct = {};
    // RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    // RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};
    uint32_t plln_val, flash_latency;

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply( PWR_LDO_SUPPLY );

    /** Configure the flash latency and main internal regulator output voltage
     *  per intended run mode (VOS0 to VOS3)
     *  VOS0: boosted performance (max frequency 480 MHz)
     *  VOS1: high performance (max frequency 400 MHz)
     *  VOS2: medium performance (max frequency 300 MHz)
     *  VOS3: low-power (max frequency 200 MHz)
    */
    if ( cfg_.cpu_freq == Config::SysClkFreq::FREQ_480MHZ ) {
        plln_val = ( hseValue == 25000000 ) ? 192 : 240;
        flash_latency = FLASH_LATENCY_4;
        __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE0 );
    } else {
        plln_val = ( hseValue == 25000000 ) ? 160 : 200;
        flash_latency = FLASH_LATENCY_2;
        __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
    }

    while ( !__HAL_PWR_GET_FLAG( PWR_FLAG_VOSRDY ) ) {}

    /* Call SystemClock_Config() in Variant file */
    SystemClock_Config( plln_val, flash_latency );

    /** Enable USB Voltage detector */
    HAL_PWREx_EnableUSBVoltageDetector();
}

void System::ConfigureMpu()
{
    MPU_Region_InitTypeDef MPU_InitStruct;
    HAL_MPU_Disable();

    // Configure 32K of RAM_D2 (aka D2 SRAM1) as non cacheable for DMA buffer use
    // We add a section to linker scripe and uvos_core.h for DMA_BUFFER_MEM_SECTION
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = 0x30000000;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_32KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // Configure the MPU attributes as Normal, Non Cacheable for
    // Flash Bank B sector 7 used as persistant config storage
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = ADDR_FLASH_SECTOR_7_BANK2;
    MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // Configure the backup SRAM region as non-cacheable
    MPU_InitStruct.IsCacheable  = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsShareable  = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number       = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.Size         = MPU_REGION_SIZE_4KB;
    MPU_InitStruct.BaseAddress  = 0x38800000;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

uint32_t System::GetSysClkFreq()
{
    return HAL_RCC_GetSysClockFreq();
}

uint32_t System::GetHClkFreq()
{
    return HAL_RCC_GetHCLKFreq();
}

uint32_t System::GetPClk1Freq()
{
    return HAL_RCC_GetPCLK1Freq();
}

uint32_t System::GetTickFreq()
{
    return HAL_RCC_GetPCLK1Freq() * 2;
}

uint32_t System::GetPClk2Freq()
{
    return HAL_RCC_GetPCLK2Freq();
}

System::MemoryRegion System::GetProgramMemoryRegion()
{
    return GetMemoryRegion(SCB->VTOR);
}

System::MemoryRegion System::GetMemoryRegion(uint32_t addr)
{
    if(addr >= D1_AXIFLASH_BASE
       && addr < D1_AXIFLASH_BASE + INTERNAL_FLASH_SIZE)
        return MemoryRegion::INTERNAL_FLASH;
    if(addr >= D1_ITCMRAM_BASE && addr < D1_ITCMRAM_BASE + ITCMRAM_SIZE)
        return MemoryRegion::ITCMRAM;
    if(addr >= D1_DTCMRAM_BASE && addr < D1_DTCMRAM_BASE + DTCMRAM_SIZE)
        return MemoryRegion::DTCMRAM;
    if(addr >= D1_AXISRAM_BASE && addr < D1_AXISRAM_BASE + SRAM_D1_SIZE)
        return MemoryRegion::SRAM_D1;
    if(addr >= D2_AXISRAM_BASE && addr < D2_AXISRAM_BASE + SRAM_D2_SIZE)
        return MemoryRegion::SRAM_D2;
    if(addr >= D3_SRAM_BASE && addr < D3_SRAM_BASE + SRAM_D3_SIZE)
        return MemoryRegion::SRAM_D3;
    if(addr >= SDRAM_BASE && addr < SDRAM_BASE + SDRAM_SIZE)
        return MemoryRegion::SDRAM;
    if(addr >= QSPI_BASE && addr < QSPI_BASE + QSPI_SIZE)
        return MemoryRegion::QSPI;

    return MemoryRegion::INVALID_ADDRESS;
}


} // namespace uvos

#else // ifndef UNIT_TEST

#include "system.h"
// this is part of the dummy version used in unit tests
TestIsolator<uvos::System::SystemState> uvos::System::testIsolator_;

#endif
