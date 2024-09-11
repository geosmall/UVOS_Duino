#pragma once

#include <vector>
#include <memory>
#include <cstdint>
#include "ProtocolParser.h"

namespace uvos
{

class SerialReceiver {
public:
    // Constructor takes a callback to handle parsed messages
    explicit SerialReceiver(SerRxParseCallback parse_callback);
    ~SerialReceiver();

    // Delete copy and move semantics of the class instance,
    // due to the DMA buffer being shared resource with uart
    SerialReceiver(const SerialReceiver&) = delete;
    SerialReceiver& operator=(const SerialReceiver&) = delete;

    // Register a protocol parser
    void register_parser(std::unique_ptr<ProtocolParser> parser);

    // Method to be called when DMA receives data
    void on_dma_receive(const uint8_t* data, size_t length);

    // Initialize DMA (to be called during system setup)
    void init_dma();

private:
    static constexpr size_t BUFFER_SIZE = 256;

    // DMA buffer placed in a specific memory section
    static DMA_BUFFER_MEM_SECTION uint8_t default_serial_rx_buffer[BUFFER_SIZE];

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
