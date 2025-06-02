#pragma once

#include "uvos_core.h"
#include "stm32h7xx_hal.h"

/* TODO:
- Add documentation
- Add IT
*/

namespace uvos
{
/** @addtogroup serial
@{
*/

/**  Handler for serial peripheral interface */
class SpiHandle
{
  public:
    struct Config
    {
        enum class Peripheral   { SPI_1, SPI_2, SPI_3, SPI_4, SPI_5, SPI_6 };
        enum class Mode         { MASTER, SLAVE };
        enum class Direction    { TWO_LINES, TWO_LINES_TX_ONLY,
                                  TWO_LINES_RX_ONLY, ONE_LINE
                                };
        enum class ClockPolarity{ LOW, HIGH };
        enum class ClockPhase   { ONE_EDGE, TWO_EDGE };
        enum class NSS          { SOFT, HARD_INPUT, HARD_OUTPUT };
        enum class NSSPulseMode { DISABLE, ENABLE };
        enum class BaudPrescaler{ PS_2,  PS_4,  PS_8,   PS_16,
                                  PS_32, PS_64, PS_128, PS_256
                                };
        struct {
            uvs_gpio_pin sclk, miso, mosi, nss;
        } pin_config;

        /* ---------- defaults ---------- */
        Config()
        {
            // user must specify periph, mode, direction, nss, and pin_config
            datasize       = 8;
            clock_polarity = ClockPolarity::LOW;
            clock_phase    = ClockPhase::ONE_EDGE;
            baud_prescaler = BaudPrescaler::PS_8;
            nss_pulse      = NSSPulseMode::DISABLE;
        }

        /* ---------- user-supplied fields ---------- */
        Peripheral    periph;
        Mode          mode;
        Direction     direction;
        unsigned long datasize;
        ClockPolarity clock_polarity;
        ClockPhase    clock_phase;
        NSS           nss;
        NSSPulseMode  nss_pulse;
        BaudPrescaler baud_prescaler;

        /* run-time computed – caller can read back but never sets */
        unsigned long disable_delay_;
    };

    /** Return values for Spi functions. */
    enum class Result
    {
        OK,          /**< & */
        ERR_TIMEOUT, /**< & */
        ERR          /**< & */
    };

    enum class DmaDirection
    {
        RX,    /**< & */
        TX,    /**< & */
        RX_TX, /**< & */
    };

    /* =================================================================
     * Construction / access
     * ================================================================= */

    /** Non-copyable: one canonical object per HW peripheral */
    SpiHandle(const SpiHandle&)            = delete;
    SpiHandle& operator=(const SpiHandle&) = delete;

    /* =================================================================
     * Public API
     * ================================================================= */

    /** Initializes handler */
    Result Init(const Config& config);

    /** Returns the current config. */
    const Config& GetConfig() const { return config_; }

    static constexpr uint32_t PrescalerToHAL(Config::BaudPrescaler baud_prescale)
    {
        switch (baud_prescale)
        {
        case Config::BaudPrescaler::PS_2:
            return SPI_BAUDRATEPRESCALER_2;
        case Config::BaudPrescaler::PS_4:
            return SPI_BAUDRATEPRESCALER_4;
        case Config::BaudPrescaler::PS_8:
            return SPI_BAUDRATEPRESCALER_8;
        case Config::BaudPrescaler::PS_16:
            return SPI_BAUDRATEPRESCALER_16;
        case Config::BaudPrescaler::PS_32:
            return SPI_BAUDRATEPRESCALER_32;
        case Config::BaudPrescaler::PS_64:
            return SPI_BAUDRATEPRESCALER_64;
        case Config::BaudPrescaler::PS_128:
            return SPI_BAUDRATEPRESCALER_128;
        case Config::BaudPrescaler::PS_256:
            return SPI_BAUDRATEPRESCALER_256;
        default:
            return 0xFFFFFFFF; // Invalid value or handle error appropriately
        }
    }

    static constexpr SPI_TypeDef* PeripheralToHAL(Config::Peripheral periph)
    {
        switch (periph)
        {
        case Config::Peripheral::SPI_1:
            return SPI1;
        case Config::Peripheral::SPI_2:
            return SPI2;
        case Config::Peripheral::SPI_3:
            return SPI3;
        case Config::Peripheral::SPI_4:
            return SPI4;
        case Config::Peripheral::SPI_5:
            return SPI5;
        case Config::Peripheral::SPI_6:
            return SPI6;
        default:
            return nullptr;
        }
    }

    /** A callback to be executed right before a dma transfer is started. */
    typedef void (*StartCallbackFunctionPtr)(void* context);
    /** A callback to be executed after a dma transfer is completed. */
    typedef void (*EndCallbackFunctionPtr)(void* context, Result result);

    /** Set SPI baud prescaler
     * \param baud_prescaler The baud prescaler to set
     * \param timeout How long to timeout for in milliseconds
     * \return Whether the baud prescaler was set successfully or not
     */
    Result SetBaudPrescaler(const Config::BaudPrescaler baud_prescaler, uint32_t timeout = 100);

    /** Set SPI baud rate to achieve a desired speed in Hz
     * \param periph The specified SPI instance
     * \param speed The desired speed in Hz
     * \param prescale_val returned value of computed prescale, passed by reference
     * \return Whether the data size was set successfully or not
     */
    Result GetBaudHz(const Config::Peripheral periph, const uint32_t speed, Config::BaudPrescaler& prescale_val);

    /** Blocking transmit 
    \param buff input buffer
    \param size  buffer size
    \param timeout how long in milliseconds the function will wait 
                   before returning without successful communication
    \return Whether the transmit was successful or not
    */
    Result BlockingTransmit(uint8_t* buff, size_t size, uint32_t timeout = 100);

    /** Polling Receive
    \param buffer input buffer
    \param size  buffer size
    \param timeout How long to timeout for in milliseconds
    \return Whether the receive was successful or not
    */
    Result BlockingReceive(uint8_t* buffer, uint16_t size, uint32_t timeout = 100);

    /** Blocking transmit and receive
    \param tx_buff the transmit buffer
    \param rx_buff the receive buffer
    \param size the length of the transaction
    \param timeout how long in milliseconds the function will wait 
                   before returning without successful communication
    */
    Result BlockingTransmitAndReceive(uint8_t* tx_buff,
                                      uint8_t* rx_buff,
                                      size_t   size,
                                      uint32_t timeout = 100);

    /** Blocking transmit and receive using LL driver
    \param tx_buff the transmit buffer
    \param rx_buff the receive buffer
    \param size the length of the transaction
    \param timeout how long in counts (not mSec) the function will wait 
                   before returning without successful communication
    */
    Result BlockingTransferLL(uint8_t* tx_buff,
                              uint8_t* rx_buff,
                              size_t   size,
                              uint32_t timeout = 1000);

    /** DMA-based transmit 
    \param *buff input buffer
    \param size  buffer size
    \param start_callback   A callback to execute when the transfer starts, or NULL.
                            The callback is called from an interrupt, so keep it fast.
    \param end_callback     A callback to execute when the transfer finishes, or NULL.
                            The callback is called from an interrupt, so keep it fast.
    \param callback_context A pointer that will be passed back to you in the callbacks.     
    \return Whether the transmit was successful or not
    */
    Result DmaTransmit(uint8_t*                            buff,
                       size_t                              size,
                       SpiHandle::StartCallbackFunctionPtr start_callback,
                       SpiHandle::EndCallbackFunctionPtr   end_callback,
                       void*                               callback_context);

    /** DMA-based receive 
    \param *buff input buffer
    \param size  buffer size
    \param start_callback   A callback to execute when the transfer starts, or NULL.
                            The callback is called from an interrupt, so keep it fast.
    \param end_callback     A callback to execute when the transfer finishes, or NULL.
                            The callback is called from an interrupt, so keep it fast.
    \param callback_context A pointer that will be passed back to you in the callbacks.    
    \return Whether the receive was successful or not
    */
    Result DmaReceive(uint8_t*                            buff,
                      size_t                              size,
                      SpiHandle::StartCallbackFunctionPtr start_callback,
                      SpiHandle::EndCallbackFunctionPtr   end_callback,
                      void*                               callback_context);

    /** DMA-based transmit and receive 
    \param tx_buff  the transmit buffer
    \param rx_buff  the receive buffer
    \param size     buffer size
    \param start_callback   A callback to execute when the transfer starts, or NULL.
                            The callback is called from an interrupt, so keep it fast.
    \param end_callback     A callback to execute when the transfer finishes, or NULL.
                            The callback is called from an interrupt, so keep it fast.
    \param callback_context A pointer that will be passed back to you in the callbacks.    
    \return Whether the receive was successful or not
    */
    Result
    DmaTransmitAndReceive(uint8_t*                            tx_buff,
                          uint8_t*                            rx_buff,
                          size_t                              size,
                          SpiHandle::StartCallbackFunctionPtr start_callback,
                          SpiHandle::EndCallbackFunctionPtr   end_callback,
                          void*                               callback_context);

    /** \return the result of HAL_SPI_GetError() to the user. */
    int CheckError();

    /* =================================================================
     * Public static interface
     * ================================================================= */
    /** Global initialization - called once at startup */
    static void GlobalInit();

    /** Obtain the unique handle for a given peripheral */
    static SpiHandle& Instance(Config::Peripheral p);

    /** Map SPI instance to handle - used by MSP callbacks */
    static SpiHandle* MapInstanceToHandle(SPI_TypeDef* instance);

    // User configuration
    Config config_;

    // Pin management for HAL MSP functions
    Result InitPins();
    Result DeInitPins();

    // HAL handles for IRQ handlers - these need to be public for MSP callbacks
    SPI_HandleTypeDef hspi_;
    DMA_HandleTypeDef hdma_spi_rx_;
    DMA_HandleTypeDef hdma_spi_tx_;

    static SpiHandle spi_handles[6];               /* canonical objects */
    static volatile int8_t dma_active_peripheral_;

    static void DmaTransferFinished(SPI_HandleTypeDef*, Result);

  private:
    /* =================================================================
     * Internal helpers
     * ================================================================= */
    SpiHandle() = default;                       /* only the static array can create */

    /* DMA scheduler -------------------------------------------------- */
    Result         SetDmaPeripheral();
    Result         InitDma();
    Result         StartDmaTx (uint8_t* tx, size_t len,
                               StartCallbackFunctionPtr, EndCallbackFunctionPtr, void*);
    Result         StartDmaRx (uint8_t* rx, size_t len,
                               StartCallbackFunctionPtr, EndCallbackFunctionPtr, void*);
    Result         StartDmaRxTx(uint8_t* tx, uint8_t* rx, size_t len,
                               StartCallbackFunctionPtr, EndCallbackFunctionPtr, void*);

    /* =================================================================
     * Static scheduler data and internal functions
     * ================================================================= */

    struct SpiDmaJob
    {
        uint8_t* data_tx = nullptr;
        uint8_t* data_rx = nullptr;
        size_t size = 0;
        DmaDirection direction = DmaDirection::TX;
        StartCallbackFunctionPtr start_callback = nullptr;
        EndCallbackFunctionPtr end_callback = nullptr;
        void* callback_context = nullptr;

        bool IsValidJob() const { return data_rx != nullptr && data_tx != nullptr; }
        void Invalidate() { data_rx = data_tx = nullptr; }
    };

    static constexpr uint8_t kNumSpiWithDma = 4;
    static SpiDmaJob queued_dma_transfers_[kNumSpiWithDma];
    static EndCallbackFunctionPtr next_end_callback_;
    static void* next_callback_context_;

    static bool IsDmaBusy();
    static void QueueDmaTransfer(size_t spi_idx, const SpiDmaJob& job);
    static bool IsDmaTransferQueuedFor(size_t spi_idx);
};

extern "C"
{
    /** internal. Used for global init. */
    void uvs_spi_global_init();
};

/** @} */
} // namespace uvos