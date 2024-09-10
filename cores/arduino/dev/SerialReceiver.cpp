#include "uvos_core.h"
#include "SerialReceiver.h"
#include "IBusParser.h"

namespace uvos
{

// Define the DMA buffer
DMA_BUFFER_MEM_SECTION uint8_t SerialReceiver::default_serial_rx_buffer[SerialReceiver::BUFFER_SIZE] = {0};

SerialReceiver::SerialReceiver(SerRxParseCallback parse_callback)
    : parse_callback_(parse_callback) {
    // Initialize DMA and UART peripherals
    init_dma();
}

SerialReceiver::~SerialReceiver() {
    // Deinitialize DMA and UART peripherals if necessary
}

void SerialReceiver::register_parser(std::unique_ptr<ProtocolParser> parser) {
    // Set the parse callback for the parser
    parser->set_parse_callback(parse_callback_);
    // Add to the list of parsers
    parsers_.emplace_back(std::move(parser));
}

void SerialReceiver::init_dma() {
    // Configure UART and DMA peripherals
    // Example using STM32 HAL:

    /*
    // Enable UART and DMA clocks
    __HAL_RCC_USARTx_CLK_ENABLE();
    __HAL_RCC_DMAx_CLK_ENABLE();

    // Configure UART handle (replace 'USARTx' and 'DMAx_Streamx' with actual instances)
    huart.Instance = USARTx;
    huart.Init.BaudRate = 115200;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart);

    // Configure DMA for UART RX
    hdma_uart_rx.Instance = DMAx_Streamx;
    hdma_uart_rx.Init.Channel = DMA_CHANNEL_y;
    hdma_uart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_uart_rx);

    // Link DMA handle to UART handle
    __HAL_LINKDMA(&huart, hdmarx, hdma_uart_rx);

    // Enable UART DMA receive
    HAL_UART_Receive_DMA(&huart, default_serial_rx_buffer, BUFFER_SIZE);

    // Configure NVIC for DMA interrupts
    HAL_NVIC_SetPriority(DMAx_Streamx_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMAx_Streamx_IRQn);
    */
    
    // Placeholder: Implement DMA initialization as per your project setup
}

void SerialReceiver::on_dma_receive(const uint8_t* data, size_t length) {
    // Iterate over each received byte and pass to all registered parsers
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        for (auto& parser : parsers_) {
            parser->parse_byte(byte);
        }
    }
}

// Example DMA IRQ handler (to be integrated with your interrupt system)
extern "C" void DMAx_Streamx_IRQHandler(void) {
    // Handle DMA interrupt
    // For example, using STM32 HAL:
    /*
    HAL_DMA_IRQHandler(&SerialReceiver::hdma_uart_rx);

    // Check for transfer complete or half-transfer
    if (__HAL_DMA_GET_IT_SOURCE(&SerialReceiver::hdma_uart_rx, DMA_IT_TC)) {
        // Clear interrupt flags
        __HAL_DMA_CLEAR_FLAG(&SerialReceiver::hdma_uart_rx, DMA_FLAG_TCIFx);
        
        // Notify SerialReceiver about received data
        SerialReceiver::on_dma_receive(SerialReceiver::default_serial_rx_buffer, SerialReceiver::BUFFER_SIZE);
    }
    */
    
    // Placeholder: Implement interrupt handling and call on_dma_receive accordingly
}

} // namespace uvos
