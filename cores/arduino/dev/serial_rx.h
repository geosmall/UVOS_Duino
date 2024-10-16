#pragma once

#include <algorithm> // std::fill
#include "per/uart.h"
#include "sys/dma.h"
#include "ProtocolParser.h"
// #include "util/FIFO.h"
#include "serial_rx/IBusParser.h"

namespace uvos
{
/** @brief   Transport layer for sending and receiving IBUS data over UART
 *  @details This is the mode of communication used for serial Rx comms
 *           There is an additional 2kB of RAM data used within this class
 *           for processing bulk data from the UART peripheral
 *  @ingroup ibus
 */
class SerialReceiver
{
  public:
    // Enum to define the receiver protocol type
    enum ProtocolType
    {
        NONE = 0,
        IBUS,
        SBUS
    };

    /** @brief Constructor for SerialReceiver object
     * @param protocol_type  Protocol type of the receiver
     */
    explicit SerialReceiver(ProtocolType protocol_type) : protocol_type_(protocol_type)
    {
        switch (protocol_type_)
        {
        case IBUS:
            parser_ = new IBusParser();
            break;
        case SBUS:
            // Initialize SBUS parser when implemented
            break;
        default:
            parser_ = nullptr;
            break;
        }
    }

    ~SerialReceiver()
    {
        if (parser_ != nullptr)
        {
            delete parser_;
            parser_ = nullptr;
        }
    }

    // Delete copy and move semantics if not needed
    SerialReceiver(const SerialReceiver&) = delete;
    SerialReceiver& operator=(const SerialReceiver&) = delete;

    /** @brief Configuration structure for UART IBUS */
    struct Config
    {
        UartHandler::Config::Peripheral periph;
        uvs_gpio_pin rx;
        uvs_gpio_pin tx;

        /** Pointer to buffer for DMA UART rx byte transfer in background.
         *
         *  @details By default this uses a shared buffer in DMA_BUFFER_MEM_SECTION,
         *           which can only be utilized for a single UART peripheral. To
         *           use IBUS with multiple UART peripherals, you must provide your own
         *           buffer, allocated to a DMA-capable memory section.
         */
        uint8_t* rx_buffer;

        /** Size in bytes of rx_buffer.
         *
         *  @details This size determines the maximum Rx bytes readable by the UART in the background.
         *           By default it's set to the size of the default shared rx_buffer (256 bytes).
         *           While much smaller sizes can be used, data can get missed if the buffer is too small.
         */
        size_t rx_buffer_size;

        Config();
    };

    /** @brief Initialization of UART using config struct */
    inline void Init(const Config config)
    {
        UartHandler::Config uart_config;

        // defaults
        uart_config.baudrate = 115200;
        uart_config.stopbits = UartHandler::Config::StopBits::BITS_1;
        uart_config.parity = UartHandler::Config::Parity::NONE;
        uart_config.mode = UartHandler::Config::Mode::TX_RX;
        uart_config.wordlength = UartHandler::Config::WordLength::BITS_8;

        // user settings
        uart_config.periph = config.periph;
        uart_config.pin_config.rx = config.rx;
        uart_config.pin_config.tx = config.tx;

        rx_buffer_ = config.rx_buffer;
        rx_buffer_size_ = config.rx_buffer_size;

        /** zero the buffer to ensure emptiness regardless of source memory */
        std::fill(rx_buffer_, rx_buffer_ + rx_buffer_size_, 0);

        uart_.Init(uart_config);
    }

    /** @brief Start the UART peripheral in listening mode */
    inline void StartRx()
    {
        // Verify that rx_buffer_ and rx_buffer_size_ are set
        if (rx_buffer_ == nullptr || rx_buffer_size_ == 0) return;

        uvs_dma_clear_cache_for_buffer(rx_buffer_, rx_buffer_size_);
        uart_.DmaListenStart(rx_buffer_, rx_buffer_size_, SerialReceiver::RxCallbackDMA, this);
    }

    /** @brief returns whether the UART peripheral is actively listening in the background or not */
    inline bool RxActive() const
    {
        return uart_.IsListening();
    }

    /** @brief Returns UART HAL UART Error Code */
    inline uint32_t GetError()
    {
        return uart_.CheckError();
    }

    /** @brief sends the buffer of bytes out of the UART peripheral */
    inline void Tx(uint8_t* buff, size_t size)
    {
        uart_.PollTx(buff, size);
    }

    // This function should be called in the main loop
    inline bool Listener() const
    {
        if (parser_ != nullptr)
        {
            return parser_->Listener();
        }
        return false;
    }

    // Retrieve a message from the receiver
    inline bool GetMessage(ParsedMessage& msg)
    {
        if (parser_ != nullptr)
        {
            return parser_->GetMessage(msg);
        }
        return false;
    }

  private:
    UartHandler uart_;
    uint8_t* rx_buffer_;
    size_t rx_buffer_size_;
    uint32_t lastGoodMessageMsec_;

    // Registered protocol parser
    ProtocolParser* parser_;

    // Protocol type
    ProtocolType protocol_type_;

    // Parsed message structure
    ParsedMessage msg;

    /** Static callback for Uart that occurs when new data
     *  is available from the peripheral.
     *
     *  TODO: Handle UartHandler errors better/at all.
     *  (If there is a UART error, there's not really any recovery
     *  option at the moment)
     */
    static void RxCallbackDMA(uint8_t* data, size_t size, void* context, UartHandler::Result res)
    {
        if (res != UartHandler::Result::OK) return; // Handle error

        SerialReceiver* self = static_cast<SerialReceiver*>(context);
        if (self->parser_)
        {
            for (size_t i = 0; i < size; ++i)
            {
                if (self->parser_->ParseByte(data[i], &self->msg))
                {
                    // Message enqueued in parser's FIFO
                }
            }
        }
    }
};

} // namespace uvos
