#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include "per/uart.h"
#include "ProtocolParser.h"

namespace uvos
{

class SerialReceiver {
public:
    // Constructor takes a callback to handle parsed messages
    // explicit SerialReceiver(SerRxParseCallback parse_callback);
    SerialReceiver();
    ~SerialReceiver();

    // Delete copy and move semantics of the class instance,
    // due to the DMA buffer being shared resource with uart
    SerialReceiver(const SerialReceiver&) = delete;
    SerialReceiver& operator=(const SerialReceiver&) = delete;

    /** @brief Configuration structure for UART IBUS */
    struct Config
    {
        UartHandler::Config::Peripheral periph;
        uvs_gpio_pin                    rx;
        uvs_gpio_pin                    tx;

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

    // Register a protocol parser
    void register_parser(std::unique_ptr<ProtocolParser> parser);

    // Method to be called when DMA receives data
    void on_dma_receive(const uint8_t* data, size_t length);

    /** Initialization of UART using config struct */
    inline void Init(const Config config);

    // Initialize DMA (to be called during system setup)
    void init_dma();

private:
    UartHandler        uart_;
    uint8_t*           rx_buffer_;
    size_t             rx_buffer_size_;

    // Collection of registered protocol parsers
    std::vector<std::unique_ptr<ProtocolParser>> parsers_;

    // Callback to notify when a message is parsed
    SerRxParseCallback parse_callback_;

    // DMA configuration details (handles, streams, etc.)
    // Example:
    // DMA_HandleTypeDef hdma_uart_rx;

    // Internal methods for DMA setup and handling
    // void configure_dma();
    // void dma_irq_handler();
};

} // namespace uvos
