#pragma once

// #include <stdint.h>
// #include <stdlib.h>
// #include <vector>
#include <memory>
// #include <algorithm>
#include "per/uart.h"
#include "sys/dma.h"
// #include "sys/system.h"
#include "ProtocolParser.h"

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
    // Constructor takes a callback to handle parsed messages
    explicit SerialReceiver(SerRxParseCallback parse_callback) : parse_callback_(parse_callback)
    {
    }

    ~SerialReceiver();

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

    inline void RegisterParser(std::unique_ptr<ProtocolParser> parser)
    {
        // Set the parse callback for the parser
        parser->set_parse_callback(parse_callback_);
        // Save away the ProtocolParser interface implementation
        parser_ = std::move(parser);
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

    /** @brief This is a no-op for UART transport - Rx is via DMA callback with circular buffer */
    inline void FlushRx()
    {
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

  private:
    UartHandler uart_;
    uint8_t* rx_buffer_;
    size_t rx_buffer_size_;
    // void*              parse_context_;

    // Registered protocol parser
    std::unique_ptr<ProtocolParser> parser_;

    // Callback to notify when a message is parsed
    SerRxParseCallback parse_callback_;

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
        if (res != UartHandler::Result::OK)
        {
            return; // Handle error
        }

        /** Read context as transport type */
        SerialReceiver* rx = reinterpret_cast<SerialReceiver*>(context);

        // Iterate over each received byte and pass to all registered parsers
        for (size_t i = 0; i < size; ++i)
        {
            uint8_t byte = data[i];
            if (rx->parser_->parse_byte(byte, &rx->msg))
            {
                if (rx->parse_callback_)
                {
                    rx->parse_callback_(rx->msg);
                }
            }
        }
    }
};

} // namespace uvos
