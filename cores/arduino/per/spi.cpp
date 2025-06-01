#include "per/spi.h"
#include "sys/system.h"
#include "util/scopedirqblocker.h"
#include "stm32h7xx_ll_spi.h"

extern "C"
{
#include "per/util/spi_util.h"
#include "util/hal_map.h"
}

// TODO
// - fix up rest of lib so that we can add a spi_handle map to the hal map
// - Add configuration for standard spi stuff.


using namespace uvos;

static void Error_Handler()
{
    asm("bkpt 255");
    while(1) {}
}

/* ==================================================================== */
/*  =====   STATIC STORAGE  =========================================== */
/* ==================================================================== */
using uvos::SpiHandle;

SpiHandle SpiHandle::spi_handles[6];            /* canonical objects */

volatile int8_t SpiHandle::dma_active_peripheral_ = -1;
SpiHandle::SpiDmaJob SpiHandle::queued_dma_transfers_[SpiHandle::kNumSpiWithDma];
SpiHandle::EndCallbackFunctionPtr SpiHandle::next_end_callback_ = nullptr;
void* SpiHandle::next_callback_context_ = nullptr;

/* ==================================================================== */
/*  =====   SpiDmaJob helpers  ======================================== */
/* ==================================================================== */
bool SpiHandle::SpiDmaJob::IsValidJob() const
{
    return data_rx != nullptr && data_tx != nullptr;
}

void SpiHandle::SpiDmaJob::Invalidate()
{
    data_rx = data_tx = nullptr;
}

/* ==================================================================== */
/*  =====   Global initialiser  ======================================= */
/* ==================================================================== */

SpiHandle* SpiHandle::MapInstanceToHandle(SPI_TypeDef* instance)
{
    // map HAL instances
    constexpr SPI_TypeDef* instances[6] = {SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};

    for(int i = 0; i < 6; i++)
    {
        if(instance == instances[i])
        {
            return &SpiHandle::spi_handles[i];
        }
    }

    /* error */
    return NULL;
}

void SpiHandle::GlobalInit()
{
    // init the scheduler queue
    dma_active_peripheral_ = -1;
    for(int per = 0; per < kNumSpiWithDma; per++)
        queued_dma_transfers_[per] = SpiHandle::SpiDmaJob();
}

/* ==================================================================== */
/*  =====   Singleton accessor  ======================================= */
/* ==================================================================== */
SpiHandle& SpiHandle::Instance(Config::Peripheral p)
{
    return spi_handles[static_cast<int>(p)];
}

/* ==================================================================== */
/*  =====   Init / de-init  =========================================== */
/* ==================================================================== */
SpiHandle::Result SpiHandle::Init(const Config& config)
{
    config_ = config;

    SPI_TypeDef* periph;
    switch(config_.periph)
    {
        case Config::Peripheral::SPI_1: periph = SPI1; break;
        case Config::Peripheral::SPI_2: periph = SPI2; break;
        case Config::Peripheral::SPI_3: periph = SPI3; break;
        case Config::Peripheral::SPI_4: periph = SPI4; break;
        case Config::Peripheral::SPI_5: periph = SPI5; break;
        case Config::Peripheral::SPI_6: periph = SPI6; break;
        default: return Result::ERR;
    }

    uint32_t mode;
    switch(config_.mode)
    {
        case Config::Mode::MASTER: mode = SPI_MODE_MASTER; break;
        case Config::Mode::SLAVE: mode = SPI_MODE_SLAVE; break;
        default: return Result::ERR;
    }

    uint32_t direction;
    switch(config_.direction)
    {
        case Config::Direction::TWO_LINES:
            direction = SPI_DIRECTION_2LINES;
            break;
        case Config::Direction::TWO_LINES_TX_ONLY:
            direction = SPI_DIRECTION_2LINES_TXONLY;
            break;
        case Config::Direction::TWO_LINES_RX_ONLY:
            direction = SPI_DIRECTION_2LINES_RXONLY;
            break;
        case Config::Direction::ONE_LINE:
            direction = SPI_DIRECTION_1LINE;
            break;
        default: return Result::ERR;
    }

    // for some reason a datasize of 30 is encoded as 29
    // ie SPI_DATASIZE_5BIT == 4, etc.
    // we might also consider going the enum route for this one, but it'll be LONG
    uint32_t datasize = config_.datasize - 1;
    if(datasize < 3 || datasize > 31)
    {
        return Result::ERR;
    }

    uint32_t clock_polarity;
    switch(config_.clock_polarity)
    {
        case Config::ClockPolarity::LOW:
            clock_polarity = SPI_POLARITY_LOW;
            break;
        case Config::ClockPolarity::HIGH:
            clock_polarity = SPI_POLARITY_HIGH;
            break;
        default: return Result::ERR;
    }

    uint32_t clock_phase;
    switch(config_.clock_phase)
    {
        case Config::ClockPhase::ONE_EDGE: clock_phase = SPI_PHASE_1EDGE; break;
        case Config::ClockPhase::TWO_EDGE: clock_phase = SPI_PHASE_2EDGE; break;
        default: return Result::ERR;
    }

    uint32_t nss;
    switch(config_.nss)
    {
        case Config::NSS::SOFT: nss = SPI_NSS_SOFT; break;
        case Config::NSS::HARD_INPUT: nss = SPI_NSS_HARD_INPUT; break;
        case Config::NSS::HARD_OUTPUT: nss = SPI_NSS_HARD_OUTPUT; break;
        default: return Result::ERR;
    }

    uint32_t nss_pulse;
    switch(config_.nss_pulse)
    {
        case Config::NSSPulseMode::DISABLE: nss_pulse = SPI_NSS_PULSE_DISABLE; break;
        case Config::NSSPulseMode::ENABLE: nss_pulse = SPI_NSS_PULSE_ENABLE; break;
        default: return Result::ERR;
    }

    uint32_t baud_prescaler;
    baud_prescaler = SpiHandle::PrescalerToHAL(config_.baud_prescaler);
    if(baud_prescaler == 0xFFFFFFFF) return Result::ERR;

    hspi_.Instance               = periph;
    hspi_.Init.Mode              = mode;
    hspi_.Init.Direction         = direction;
    hspi_.Init.DataSize          = datasize;
    hspi_.Init.CLKPolarity       = clock_polarity;
    hspi_.Init.CLKPhase          = clock_phase;
    hspi_.Init.NSS               = nss;
    hspi_.Init.BaudRatePrescaler = baud_prescaler;
    hspi_.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi_.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi_.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi_.Init.CRCPolynomial     = 0x0;
    hspi_.Init.NSSPMode          = nss_pulse;
    hspi_.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    hspi_.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    hspi_.Init.TxCRCInitializationPattern
        = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi_.Init.RxCRCInitializationPattern
        = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi_.Init.MasterSSIdleness        = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi_.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_01CYCLE;
    hspi_.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi_.Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi_.Init.IOSwap                  = SPI_IO_SWAP_DISABLE;
    if(HAL_SPI_Init(&hspi_) != HAL_OK)
    {
        Error_Handler();
        return SpiHandle::Result::ERR;
    }

    // Compute delay used below in BlockingTransferLL()
    config_.disable_delay_ = spi_compute_disable_delay_us(hspi_.Instance);

    return SpiHandle::Result::OK;
}

SpiHandle::Result SpiHandle::SetBaudPrescaler(const Config::BaudPrescaler baud_prescale, uint32_t timeout)
{

    // Check that HAL SPI peripheral if handle is valid
    if (!(IS_SPI_ALL_INSTANCE(hspi_.Instance))) return SpiHandle::Result::ERR;

    // Map the BaudPrescaler enum to the HAL version
    uint32_t hal_prescaler = SpiHandle::PrescalerToHAL(baud_prescale);
    if (hal_prescaler == 0xFFFFFFFF) return Result::ERR;

    // Loop if SPI is busy, return error if timeout is reached
    uint32_t tickstart = HAL_GetTick();
    while (HAL_SPI_GetState(&hspi_) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > timeout) return SpiHandle::Result::ERR;
    }

    // Get the SPI peripheral from the HAL handle
    SPI_TypeDef* SPIx = hspi_.Instance;
    // Modify the prescaler bits in SPIx->CFG1 register
    MODIFY_REG(SPIx->CFG1, SPI_CFG1_MBR_Msk, hal_prescaler);

    return SpiHandle::Result::OK;
}

SpiHandle::Result SpiHandle::GetBaudHz(const Config::Peripheral periph,
                                             const uint32_t speed,
                                             Config::BaudPrescaler& prescale_val)
{
    SPI_TypeDef* instance = PeripheralToHAL(periph);

    // Check that HAL SPI peripheral instance is valid
    if (!(IS_SPI_ALL_INSTANCE(instance))) return SpiHandle::Result::ERR;

    uint32_t spi_freq = spi_get_clk_freq_inst(instance);

    // clang-format off
    if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
        prescale_val = Config::BaudPrescaler::PS_2;
    } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
        prescale_val = Config::BaudPrescaler::PS_4;
    } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
        prescale_val = Config::BaudPrescaler::PS_8;
    } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
        prescale_val = Config::BaudPrescaler::PS_16;
    } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
        prescale_val = Config::BaudPrescaler::PS_32;
    } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
        prescale_val = Config::BaudPrescaler::PS_64;
    } else if (speed >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
        prescale_val = Config::BaudPrescaler::PS_128;
    } else {
        prescale_val = Config::BaudPrescaler::PS_256;
    }
    // clang-format on

    return SpiHandle::Result::OK;
}

SpiHandle::Result SpiHandle::SetDmaPeripheral()
{
    switch(config_.periph)
    {
        case SpiHandle::Config::Peripheral::SPI_1:
            hdma_spi_rx_.Init.Request = DMA_REQUEST_SPI1_RX;
            hdma_spi_tx_.Init.Request = DMA_REQUEST_SPI1_TX;
            break;
        case SpiHandle::Config::Peripheral::SPI_2:
            hdma_spi_rx_.Init.Request = DMA_REQUEST_SPI2_RX;
            hdma_spi_tx_.Init.Request = DMA_REQUEST_SPI2_TX;
            break;
        case SpiHandle::Config::Peripheral::SPI_3:
            hdma_spi_rx_.Init.Request = DMA_REQUEST_SPI3_RX;
            hdma_spi_tx_.Init.Request = DMA_REQUEST_SPI3_TX;
            break;
        case SpiHandle::Config::Peripheral::SPI_4:
            hdma_spi_rx_.Init.Request = DMA_REQUEST_SPI4_RX;
            hdma_spi_tx_.Init.Request = DMA_REQUEST_SPI4_TX;
            break;
        case SpiHandle::Config::Peripheral::SPI_5:
            hdma_spi_rx_.Init.Request = DMA_REQUEST_SPI5_RX;
            hdma_spi_tx_.Init.Request = DMA_REQUEST_SPI5_TX;
            break;
        // DMA_REQUEST_SPI6_TX is not available?
        default: return SpiHandle::Result::ERR;
    }
    return SpiHandle::Result::OK;
}

SpiHandle::Result SpiHandle::InitDma()
{
    hdma_spi_rx_.Instance                 = DMA2_Stream2;
    hdma_spi_rx_.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi_rx_.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi_rx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_rx_.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi_rx_.Init.Mode                = DMA_NORMAL;
    hdma_spi_rx_.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_spi_rx_.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_spi_rx_.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_spi_rx_.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_spi_rx_.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    hdma_spi_tx_.Instance                 = DMA2_Stream3;
    hdma_spi_tx_.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spi_tx_.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spi_tx_.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_tx_.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_spi_tx_.Init.Mode                = DMA_NORMAL;
    hdma_spi_tx_.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_spi_tx_.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_spi_tx_.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_spi_tx_.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_spi_tx_.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    SetDmaPeripheral();

    hdma_spi_rx_.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_tx_.Init.Direction = DMA_MEMORY_TO_PERIPH;

    if(HAL_DMA_Init(&hdma_spi_rx_) != HAL_OK)
    {
        Error_Handler();
        return SpiHandle::Result::ERR;
    }
    if(HAL_DMA_Init(&hdma_spi_tx_) != HAL_OK)
    {
        Error_Handler();
        return SpiHandle::Result::ERR;
    }

    __HAL_LINKDMA(&hspi_, hdmarx, hdma_spi_rx_);
    __HAL_LINKDMA(&hspi_, hdmatx, hdma_spi_tx_);

    return SpiHandle::Result::OK;
}


void SpiHandle::DmaTransferFinished(SPI_HandleTypeDef* hspi,
                                          SpiHandle::Result  result)
{
    ScopedIrqBlocker block;

    // on an error, reinit the peripheral to clear any flags
    if(result != SpiHandle::Result::OK)
        HAL_SPI_Init(hspi);

    dma_active_peripheral_ = -1;

    if(next_end_callback_ != nullptr)
    {
        // the callback may setup another transmission, hence we shouldn't reset this to
        // nullptr after the callback - it might overwrite the new transmission.
        auto callback      = next_end_callback_;
        next_end_callback_ = nullptr;
        // make the callback
        callback(next_callback_context_, result);
    }

    // the callback could have started a new transmission right away...
    if(IsDmaBusy())
        return;

    // dma is still idle. Check if another SPI peripheral waits for a job.
    for(int per = 0; per < kNumSpiWithDma; per++)
        if(IsDmaTransferQueuedFor(per))
        {
            SpiHandle::Result result;
            if(queued_dma_transfers_[per].direction
               == SpiHandle::DmaDirection::TX)
            {
                result = spi_handles[per].StartDmaTx(
                    queued_dma_transfers_[per].data_tx,
                    queued_dma_transfers_[per].size,
                    queued_dma_transfers_[per].start_callback,
                    queued_dma_transfers_[per].end_callback,
                    queued_dma_transfers_[per].callback_context);
            }
            else if(queued_dma_transfers_[per].direction
                    == SpiHandle::DmaDirection::RX)
            {
                result = spi_handles[per].StartDmaRx(
                    queued_dma_transfers_[per].data_rx,
                    queued_dma_transfers_[per].size,
                    queued_dma_transfers_[per].start_callback,
                    queued_dma_transfers_[per].end_callback,
                    queued_dma_transfers_[per].callback_context);
            }
            else
            {
                result = spi_handles[per].StartDmaRxTx(
                    queued_dma_transfers_[per].data_rx,
                    queued_dma_transfers_[per].data_tx,
                    queued_dma_transfers_[per].size,
                    queued_dma_transfers_[per].start_callback,
                    queued_dma_transfers_[per].end_callback,
                    queued_dma_transfers_[per].callback_context);
            }
            if(result == SpiHandle::Result::OK)
            {
                // remove the job from the queue
                queued_dma_transfers_[per].Invalidate();
                return;
            }
        }
}

int SpiHandle::CheckError()
{
    return HAL_SPI_GetError(&hspi_);
}


bool SpiHandle::IsDmaBusy()
{
    return dma_active_peripheral_ >= 0;
}

bool SpiHandle::IsDmaTransferQueuedFor(size_t spi_idx)
{
    return queued_dma_transfers_[spi_idx].IsValidJob();
}

void SpiHandle::QueueDmaTransfer(size_t spi_idx, const SpiDmaJob& job)
{
    // wait for any previous job on this peripheral to finish
    // and the queue position to become free
    while(IsDmaTransferQueuedFor(spi_idx))
    {
        continue;
    };

    // queue the job
    ScopedIrqBlocker block;
    queued_dma_transfers_[spi_idx] = job;
}

SpiHandle::Result
SpiHandle::DmaTransmit(uint8_t*                            buff,
                       size_t                              size,
                       SpiHandle::StartCallbackFunctionPtr start_callback,
                       SpiHandle::EndCallbackFunctionPtr   end_callback,
                       void* callback_context)
{
    // if dma is currently running - queue a job
    if(IsDmaBusy())
    {
        SpiDmaJob job;
        job.data_tx          = buff;
        job.size             = size;
        job.direction        = SpiHandle::DmaDirection::TX;
        job.start_callback   = start_callback;
        job.end_callback     = end_callback;
        job.callback_context = callback_context;

        const int spi_idx = int(config_.periph);

        // queue a job (blocks until the queue position is free)
        QueueDmaTransfer(spi_idx, job);
        // TODO: the user can't tell if he got returned "OK"
        // because the transfer was executed or because it was queued...
        // should we change that?
        return SpiHandle::Result::OK;
    }

    return StartDmaTx(
        buff, size, start_callback, end_callback, callback_context);
}

SpiHandle::Result
SpiHandle::StartDmaTx(uint8_t*                            buff,
                      size_t                              size,
                      SpiHandle::StartCallbackFunctionPtr start_callback,
                      SpiHandle::EndCallbackFunctionPtr   end_callback,
                      void* callback_context)
{
    while(HAL_SPI_GetState(&hspi_) != HAL_SPI_STATE_READY) {};

    if(InitDma() != SpiHandle::Result::OK)
    {
        if(end_callback)
            end_callback(callback_context, SpiHandle::Result::ERR);
        return SpiHandle::Result::ERR;
    }

    ScopedIrqBlocker block;

    dma_active_peripheral_ = int(config_.periph);
    next_end_callback_     = end_callback;
    next_callback_context_ = callback_context;

    if(start_callback)
        start_callback(callback_context);

    if(HAL_SPI_Transmit_DMA(&hspi_, buff, size) != HAL_OK)
    {
        dma_active_peripheral_ = -1;
        next_end_callback_     = NULL;
        next_callback_context_ = NULL;
        if(end_callback)
            end_callback(callback_context, SpiHandle::Result::ERR);
        return SpiHandle::Result::ERR;
    }
    return SpiHandle::Result::OK;
}

SpiHandle::Result
SpiHandle::DmaReceive(uint8_t*                            buff,
                      size_t                              size,
                      SpiHandle::StartCallbackFunctionPtr start_callback,
                      SpiHandle::EndCallbackFunctionPtr   end_callback,
                      void* callback_context)
{
    // if dma is currently running - queue a job
    if(IsDmaBusy())
    {
        SpiDmaJob job;
        job.data_rx          = buff;
        job.size             = size;
        job.direction        = SpiHandle::DmaDirection::RX;
        job.start_callback   = start_callback;
        job.end_callback     = end_callback;
        job.callback_context = callback_context;

        const int spi_idx = int(config_.periph);

        // queue a job (blocks until the queue position is free)
        QueueDmaTransfer(spi_idx, job);
        // TODO: the user can't tell if he got returned "OK"
        // because the transfer was executed or because it was queued...
        // should we change that?
        return SpiHandle::Result::OK;
    }

    return StartDmaRx(
        buff, size, start_callback, end_callback, callback_context);
}

SpiHandle::Result
SpiHandle::StartDmaRx(uint8_t*                            buff,
                      size_t                              size,
                      SpiHandle::StartCallbackFunctionPtr start_callback,
                      SpiHandle::EndCallbackFunctionPtr   end_callback,
                      void* callback_context)
{
    while(HAL_SPI_GetState(&hspi_) != HAL_SPI_STATE_READY) {};

    if(InitDma() != SpiHandle::Result::OK)
    {
        if(end_callback)
            end_callback(callback_context, SpiHandle::Result::ERR);
        return SpiHandle::Result::ERR;
    }

    ScopedIrqBlocker block;

    dma_active_peripheral_ = int(config_.periph);
    next_end_callback_     = end_callback;
    next_callback_context_ = callback_context;

    if(start_callback)
        start_callback(callback_context);

    if(HAL_SPI_Receive_DMA(&hspi_, buff, size) != HAL_OK)
    {
        dma_active_peripheral_ = -1;
        next_end_callback_     = NULL;
        next_callback_context_ = NULL;
        if(end_callback)
            end_callback(callback_context, SpiHandle::Result::ERR);
        return SpiHandle::Result::ERR;
    }
    return SpiHandle::Result::OK;
}

SpiHandle::Result SpiHandle::DmaTransmitAndReceive(
    uint8_t*                            tx_buff,
    uint8_t*                            rx_buff,
    size_t                              size,
    SpiHandle::StartCallbackFunctionPtr start_callback,
    SpiHandle::EndCallbackFunctionPtr   end_callback,
    void*                               callback_context)
{
    // if dma is currently running - queue a job
    if(IsDmaBusy())
    {
        SpiDmaJob job;
        job.data_rx          = rx_buff;
        job.data_tx          = tx_buff;
        job.size             = size;
        job.direction        = SpiHandle::DmaDirection::RX_TX;
        job.start_callback   = start_callback;
        job.end_callback     = end_callback;
        job.callback_context = callback_context;

        const int spi_idx = int(config_.periph);

        // queue a job (blocks until the queue position is free)
        QueueDmaTransfer(spi_idx, job);
        // TODO: the user can't tell if he got returned "OK"
        // because the transfer was executed or because it was queued...
        // should we change that?
        return SpiHandle::Result::OK;
    }

    return StartDmaRxTx(
        rx_buff, tx_buff, size, start_callback, end_callback, callback_context);
}

SpiHandle::Result SpiHandle::StartDmaRxTx(
    uint8_t*                            rx_buff,
    uint8_t*                            tx_buff,
    size_t                              size,
    SpiHandle::StartCallbackFunctionPtr start_callback,
    SpiHandle::EndCallbackFunctionPtr   end_callback,
    void*                               callback_context)
{
    while(HAL_SPI_GetState(&hspi_) != HAL_SPI_STATE_READY) {};

    if(InitDma() != SpiHandle::Result::OK)
    {
        if(end_callback)
            end_callback(callback_context, SpiHandle::Result::ERR);
        return SpiHandle::Result::ERR;
    }

    ScopedIrqBlocker block;

    dma_active_peripheral_ = int(config_.periph);
    next_end_callback_     = end_callback;
    next_callback_context_ = callback_context;

    if(start_callback)
        start_callback(callback_context);

    if(HAL_SPI_TransmitReceive_DMA(&hspi_, tx_buff, rx_buff, size) != HAL_OK)
    {
        dma_active_peripheral_ = -1;
        next_end_callback_     = NULL;
        next_callback_context_ = NULL;
        if(end_callback)
            end_callback(callback_context, SpiHandle::Result::ERR);
        return SpiHandle::Result::ERR;
    }
    return SpiHandle::Result::OK;
}

SpiHandle::Result
SpiHandle::BlockingTransmit(uint8_t* buff, size_t size, uint32_t timeout)
{
    if(HAL_SPI_Transmit(&hspi_, buff, size, timeout) != HAL_OK)
    {
        return SpiHandle::Result::ERR;
    }
    return SpiHandle::Result::OK;
}

SpiHandle::Result SpiHandle::BlockingReceive(uint8_t* buffer,
                                                   uint16_t size,
                                                   uint32_t timeout)
{
    if(HAL_SPI_Receive(&hspi_, buffer, size, timeout) != HAL_OK)
    {
        return Result::ERR;
    }
    return Result::OK;
}

SpiHandle::Result SpiHandle::BlockingTransmitAndReceive(uint8_t* tx_buff,
                                                        uint8_t* rx_buff,
                                                        size_t   size,
                                                        uint32_t timeout)
{
    if(HAL_SPI_TransmitReceive(&hspi_, tx_buff, rx_buff, size, timeout)
       != HAL_OK)
    {
        return SpiHandle::Result::ERR;
    }
    return SpiHandle::Result::OK;
}

SpiHandle::Result SpiHandle::BlockingTransferLL(uint8_t* tx_buffer,
                                                uint8_t* rx_buffer,
                                                size_t   len,
                                                uint32_t Timeout)

{
    /*------------------------------------------------------------------
     * Guard-bytes used when the caller does “read-only” (tx == nullptr)
     * or “write-only” (rx == nullptr).  They live in .bss so the address
     * is always valid and 32-bit aligned.
     *------------------------------------------------------------------*/
    static uint8_t dummy_tx = 0xFF;   // clocks idle-high data
    static uint8_t dummy_rx;

    if ((len == 0U) || (Timeout == 0U))
        return SpiHandle::Result::ERR;

    if (tx_buffer == nullptr) tx_buffer = &dummy_tx; // protect against dereferenced nullptr
    if (rx_buffer == nullptr) rx_buffer = &dummy_rx; // discard bytes safely

    SPI_TypeDef *const _SPI = hspi_.Instance;
    uint32_t tickstart      = HAL_GetTick();
    SpiHandle::Result ret   = SpiHandle::Result::OK;

    LL_SPI_SetTransferSize(_SPI, len);
    LL_SPI_Enable(_SPI);
    LL_SPI_StartMasterTransfer(_SPI);

    while (len--)
    {
        while (!LL_SPI_IsActiveFlag_TXP(_SPI));

        // LL_SPI_TransmitData8(_SPI, *tx_buffer++);    // safe: tx_buffer valid
        uint8_t tx_byte = (tx_buffer == &dummy_tx) ? dummy_tx : *tx_buffer++;
        LL_SPI_TransmitData8(_SPI, tx_byte);

        while (!LL_SPI_IsActiveFlag_RXP(_SPI))
        {
            if ((Timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - tickstart) >= Timeout))
            {
                ret = SpiHandle::Result::ERR_TIMEOUT;
                goto transfer_exit;
            }
        }

        // *rx_buffer++ = LL_SPI_ReceiveData8(_SPI);    // safe: rx_buffer valid
        uint8_t rx_byte = LL_SPI_ReceiveData8(_SPI);
        if (rx_buffer != &dummy_rx) *rx_buffer++ = rx_byte;
    }

transfer_exit:

    while (!LL_SPI_IsActiveFlag_EOT(_SPI));
    // Add a delay before disabling SPI otherwise last-bit/last-clock may be truncated
    // See https://github.com/stm32duino/Arduino_Core_STM32/issues/1294
    // Computed delay is half SPI clock
    System::DelayUs(config_.disable_delay_);

    /* Close transfer */
    /* Clear flags */
    LL_SPI_ClearFlag_EOT(_SPI);
    LL_SPI_ClearFlag_TXTF(_SPI);
    /* Disable SPI peripheral */
    LL_SPI_Disable(_SPI);

    return ret;
}

typedef struct
{
    uvs_gpio_pin pin;
    uint8_t      alt;
} pin_alt_spi;

static pin_alt_spi pins_none_spi = {{UVS_GPIOX, 0}, 255};

//this is a bit long...
/* ============== spi1 ============== */
static pin_alt_spi spi1_pins_sclk[] = {{{UVS_GPIOG, 11}, GPIO_AF5_SPI1},
                                       {{UVS_GPIOA, 5}, GPIO_AF5_SPI1},
                                       pins_none_spi};

static pin_alt_spi spi1_pins_miso[] = {{{UVS_GPIOB, 4}, GPIO_AF5_SPI1},
                                       {{UVS_GPIOA, 6}, GPIO_AF5_SPI1},
                                       {{UVS_GPIOG, 9}, GPIO_AF5_SPI1}};

static pin_alt_spi spi1_pins_mosi[] = {{{UVS_GPIOB, 5}, GPIO_AF5_SPI1},
                                       {{UVS_GPIOA, 7}, GPIO_AF5_SPI1},
                                       {{UVS_GPIOD, 7}, GPIO_AF5_SPI1}};

static pin_alt_spi spi1_pins_nss[] = {{{UVS_GPIOG, 10}, GPIO_AF5_SPI1},
                                      {{UVS_GPIOA, 4}, GPIO_AF5_SPI1},
                                      pins_none_spi};

/* ============== spi2 ============== */
static pin_alt_spi spi2_pins_sclk[]
    = {{{UVS_GPIOD, 3}, GPIO_AF5_SPI2}, pins_none_spi, pins_none_spi};

static pin_alt_spi spi2_pins_miso[] = {{{UVS_GPIOB, 14}, GPIO_AF5_SPI2},
                                       {{UVS_GPIOC, 2}, GPIO_AF5_SPI2},
                                       pins_none_spi};

static pin_alt_spi spi2_pins_mosi[] = {{{UVS_GPIOC, 1}, GPIO_AF5_SPI2},
                                       {{UVS_GPIOB, 15}, GPIO_AF5_SPI2},
                                       {{UVS_GPIOC, 3}, GPIO_AF5_SPI2}};

static pin_alt_spi spi2_pins_nss[] = {{{UVS_GPIOB, 12}, GPIO_AF5_SPI2},
                                      {{UVS_GPIOB, 4}, GPIO_AF7_SPI2},
                                      {{UVS_GPIOB, 9}, GPIO_AF5_SPI2}};

/* ============== spi3 ============== */
static pin_alt_spi spi3_pins_sclk[]
    = {{{UVS_GPIOC, 10}, GPIO_AF6_SPI3}, pins_none_spi, pins_none_spi};

static pin_alt_spi spi3_pins_miso[] = {{{UVS_GPIOC, 11}, GPIO_AF6_SPI3},
                                       {{UVS_GPIOB, 4}, GPIO_AF6_SPI3},
                                       pins_none_spi};

static pin_alt_spi spi3_pins_mosi[] = {{{UVS_GPIOC, 12}, GPIO_AF6_SPI3},
                                       {{UVS_GPIOB, 5}, GPIO_AF7_SPI3},
                                       pins_none_spi};

static pin_alt_spi spi3_pins_nss[]
    = {{{UVS_GPIOA, 4}, GPIO_AF6_SPI3}, pins_none_spi, pins_none_spi};

/* ============== spi4 ============== */
static pin_alt_spi spi4_pins_sclk[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

static pin_alt_spi spi4_pins_miso[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

static pin_alt_spi spi4_pins_mosi[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

static pin_alt_spi spi4_pins_nss[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

/* ============== spi5 ============== */
static pin_alt_spi spi5_pins_sclk[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

static pin_alt_spi spi5_pins_miso[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

static pin_alt_spi spi5_pins_mosi[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

static pin_alt_spi spi5_pins_nss[]
    = {pins_none_spi, pins_none_spi, pins_none_spi};

/* ============== spi6 ============== */
static pin_alt_spi spi6_pins_sclk[]
    = {{{UVS_GPIOA, 5}, GPIO_AF8_SPI6}, pins_none_spi, pins_none_spi};

static pin_alt_spi spi6_pins_miso[] = {{{UVS_GPIOB, 4}, GPIO_AF8_SPI6},
                                       {{UVS_GPIOA, 6}, GPIO_AF8_SPI6},
                                       pins_none_spi};

static pin_alt_spi spi6_pins_mosi[] = {{{UVS_GPIOB, 5}, GPIO_AF8_SPI6},
                                       {{UVS_GPIOA, 7}, GPIO_AF8_SPI6},
                                       pins_none_spi};

static pin_alt_spi spi6_pins_nss[]
    = {{{UVS_GPIOA, 4}, GPIO_AF8_SPI6}, pins_none_spi, pins_none_spi};

//an array to hold everything
static pin_alt_spi* pins_periphs_spi[] = {
    spi1_pins_sclk, spi1_pins_miso, spi1_pins_mosi, spi1_pins_nss,
    spi2_pins_sclk, spi2_pins_miso, spi2_pins_mosi, spi2_pins_nss,
    spi3_pins_sclk, spi3_pins_miso, spi3_pins_mosi, spi3_pins_nss,
    spi4_pins_sclk, spi4_pins_miso, spi4_pins_mosi, spi4_pins_nss,
    spi5_pins_sclk, spi5_pins_miso, spi5_pins_mosi, spi5_pins_nss,
    spi6_pins_sclk, spi6_pins_miso, spi6_pins_mosi, spi6_pins_nss,
};

SpiHandle::Result
checkPinMatchSpi(GPIO_InitTypeDef* init, uvs_gpio_pin pin, int p_num)
{
    for(int i = 0; i < 3; i++)
    {
        if(uvs_pin_cmp(&pins_periphs_spi[p_num][i].pin, &pins_none_spi.pin))
        {
            /* skip */
        }

        else if(uvs_pin_cmp(&pins_periphs_spi[p_num][i].pin, &pin))
        {
            init->Alternate = pins_periphs_spi[p_num][i].alt;
            return SpiHandle::Result::OK;
        }
    }

    return SpiHandle::Result::ERR;
}

SpiHandle::Result SpiHandle::InitPins()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    int per_num = 4 * (int)(config_.periph);

    bool is_master = config_.mode == Config::Mode::MASTER;

    //  slave all except 2linerx, master 2line and 2linerx
    bool enable_miso
        = (!is_master
           && config_.direction != Config::Direction::TWO_LINES_RX_ONLY)
          || (is_master
              && config_.direction == Config::Direction::TWO_LINES_RX_ONLY)
          || (is_master && config_.direction == Config::Direction::TWO_LINES);

    //  master all except 2linerx, slave 2linerx and 2line
    bool enable_mosi
        = (is_master
           && config_.direction != Config::Direction::TWO_LINES_RX_ONLY)
          || (!is_master
              && config_.direction == Config::Direction::TWO_LINES_RX_ONLY)
          || (!is_master && config_.direction == Config::Direction::TWO_LINES);

    // nss = soft -> ss pin is unused for master and slave
    bool enable_ss = config_.nss != Config::NSS::SOFT;

    if(config_.pin_config.sclk.port != UVS_GPIOX)
    {
        //check sclk against periph
        if(checkPinMatchSpi(&GPIO_InitStruct, config_.pin_config.sclk, per_num)
           == Result::ERR)
        {
            return Result::ERR;
        }

        //setup sclk pin
        GPIO_TypeDef* port  = uvs_hal_map_get_port(&config_.pin_config.sclk);
        GPIO_InitStruct.Pin = uvs_hal_map_get_pin(&config_.pin_config.sclk);
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    }

    if(config_.pin_config.miso.port != UVS_GPIOX && enable_miso)
    {
        //check miso against periph
        if(checkPinMatchSpi(
               &GPIO_InitStruct, config_.pin_config.miso, per_num + 1)
           == Result::ERR)
        {
            return Result::ERR;
        }

        //setup miso pin
        GPIO_TypeDef* port  = uvs_hal_map_get_port(&config_.pin_config.miso);
        GPIO_InitStruct.Pin = uvs_hal_map_get_pin(&config_.pin_config.miso);
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    }

    if(config_.pin_config.mosi.port != UVS_GPIOX && enable_mosi)
    {
        //check mosi against periph
        if(checkPinMatchSpi(
               &GPIO_InitStruct, config_.pin_config.mosi, per_num + 2)
           == Result::ERR)
        {
            return Result::ERR;
        }

        //setup mosi pin
        GPIO_TypeDef* port  = uvs_hal_map_get_port(&config_.pin_config.mosi);
        GPIO_InitStruct.Pin = uvs_hal_map_get_pin(&config_.pin_config.mosi);
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    }

    if(config_.pin_config.nss.port != UVS_GPIOX && enable_ss)
    {
        //check nss against periph
        if(checkPinMatchSpi(
               &GPIO_InitStruct, config_.pin_config.nss, per_num + 3)
           == Result::ERR)
        {
            return Result::ERR;
        }

        //setup nss pin
        GPIO_TypeDef* port  = uvs_hal_map_get_port(&config_.pin_config.nss);
        GPIO_InitStruct.Pin = uvs_hal_map_get_pin(&config_.pin_config.nss);
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    }

    return Result::OK;
}

SpiHandle::Result SpiHandle::DeInitPins()
{
    GPIO_TypeDef* port = uvs_hal_map_get_port(&config_.pin_config.sclk);
    uint16_t      pin  = uvs_hal_map_get_pin(&config_.pin_config.sclk);
    HAL_GPIO_DeInit(port, pin);

    port = uvs_hal_map_get_port(&config_.pin_config.miso);
    pin  = uvs_hal_map_get_pin(&config_.pin_config.miso);
    HAL_GPIO_DeInit(port, pin);

    port = uvs_hal_map_get_port(&config_.pin_config.mosi);
    pin  = uvs_hal_map_get_pin(&config_.pin_config.mosi);
    HAL_GPIO_DeInit(port, pin);

    port = uvs_hal_map_get_port(&config_.pin_config.nss);
    pin  = uvs_hal_map_get_pin(&config_.pin_config.nss);
    HAL_GPIO_DeInit(port, pin);

    return Result::OK;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
    SpiHandle* handle = SpiHandle::MapInstanceToHandle(spiHandle->Instance);

    //enable clock on all 4 pins
    uvs_hal_map_gpio_clk_enable(handle->config_.pin_config.sclk.port);
    uvs_hal_map_gpio_clk_enable(handle->config_.pin_config.miso.port);
    uvs_hal_map_gpio_clk_enable(handle->config_.pin_config.mosi.port);
    uvs_hal_map_gpio_clk_enable(handle->config_.pin_config.nss.port);

    //enable clock for our peripheral
    switch(handle->config_.periph)
    {
        case SpiHandle::Config::Peripheral::SPI_1:
        {
            __HAL_RCC_SPI1_CLK_ENABLE();
            HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SPI1_IRQn);
        }
        break;
        case SpiHandle::Config::Peripheral::SPI_2:
        {
            __HAL_RCC_SPI2_CLK_ENABLE();
            HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SPI2_IRQn);
        }
        break;
        case SpiHandle::Config::Peripheral::SPI_3:
        {
            __HAL_RCC_SPI3_CLK_ENABLE();
            HAL_NVIC_SetPriority(SPI3_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SPI3_IRQn);
        }
        break;
        case SpiHandle::Config::Peripheral::SPI_4:
        {
            __HAL_RCC_SPI4_CLK_ENABLE();
            HAL_NVIC_SetPriority(SPI4_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SPI4_IRQn);
        }
        break;
        case SpiHandle::Config::Peripheral::SPI_5:
        {
            __HAL_RCC_SPI5_CLK_ENABLE();
            HAL_NVIC_SetPriority(SPI5_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(SPI5_IRQn);
        }
        break;
        case SpiHandle::Config::Peripheral::SPI_6:
            __HAL_RCC_SPI6_CLK_ENABLE();
            break;
    }


    if(handle->InitPins() == SpiHandle::Result::ERR)
    {
        Error_Handler();
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
    SpiHandle* handle = SpiHandle::MapInstanceToHandle(spiHandle->Instance);

    //disable clock for our peripheral
    switch(handle->config_.periph)
    {
        case SpiHandle::Config::Peripheral::SPI_1:
            __HAL_RCC_SPI1_CLK_DISABLE();
            break;
        case SpiHandle::Config::Peripheral::SPI_2:
            __HAL_RCC_SPI2_CLK_DISABLE();
            break;
        case SpiHandle::Config::Peripheral::SPI_3:
            __HAL_RCC_SPI3_CLK_DISABLE();
            break;
        case SpiHandle::Config::Peripheral::SPI_4:
            __HAL_RCC_SPI4_CLK_DISABLE();
            break;
        case SpiHandle::Config::Peripheral::SPI_5:
            __HAL_RCC_SPI5_CLK_DISABLE();
            break;
        case SpiHandle::Config::Peripheral::SPI_6:
            __HAL_RCC_SPI6_CLK_DISABLE();
            break;
    }

    if(handle->DeInitPins() == SpiHandle::Result::ERR)
    {
        Error_Handler();
    }
}

extern "C" void uvs_spi_global_init()
{
    SpiHandle::GlobalInit();
}

extern "C" void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&SpiHandle::spi_handles[0].hspi_);
}

extern "C" void SPI2_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&SpiHandle::spi_handles[1].hspi_);
}

extern "C" void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&SpiHandle::spi_handles[2].hspi_);
}

extern "C" void SPI4_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&SpiHandle::spi_handles[3].hspi_);
}

extern "C" void SPI5_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&SpiHandle::spi_handles[4].hspi_);
}

void HalSpiDmaRxStreamCallback(void)
{
    ScopedIrqBlocker block;
    if(SpiHandle::dma_active_peripheral_ >= 0)
        HAL_DMA_IRQHandler(
            &SpiHandle::spi_handles[SpiHandle::dma_active_peripheral_].hdma_spi_rx_);
}
extern "C" void DMA2_Stream2_IRQHandler(void)
{
    HalSpiDmaRxStreamCallback();
}
void HalSpiDmaTxStreamCallback(void)
{
    ScopedIrqBlocker block;
    if(SpiHandle::dma_active_peripheral_ >= 0)
        HAL_DMA_IRQHandler(
            &SpiHandle::spi_handles[SpiHandle::dma_active_peripheral_].hdma_spi_tx_);
}
extern "C" void DMA2_Stream3_IRQHandler(void)
{
    HalSpiDmaTxStreamCallback();
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SpiHandle::DmaTransferFinished(hspi, SpiHandle::Result::OK);
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SpiHandle::DmaTransferFinished(hspi, SpiHandle::Result::OK);
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    SpiHandle::DmaTransferFinished(hspi, SpiHandle::Result::OK);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    SpiHandle::DmaTransferFinished(hspi, SpiHandle::Result::ERR);
}
