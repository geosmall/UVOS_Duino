#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <algorithm>
#include "per/uart.h"
#include "sys/dma.h"
#include "sys/system.h"
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
    typedef void (*SerRxParseCallback)(uint8_t* data,
                                        size_t   size,
                                        void*    context);

    SerialReceiver() {}
    ~SerialReceiver() {}

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

    /** @brief Initialization of UART using config struct */
    inline void Init(const Config config)
    {
        UartHandler::Config uart_config;

        // defaults
        uart_config.baudrate   = 115200;
        uart_config.stopbits   = UartHandler::Config::StopBits::BITS_1;
        uart_config.parity     = UartHandler::Config::Parity::NONE;
        uart_config.mode       = UartHandler::Config::Mode::TX_RX;
        uart_config.wordlength = UartHandler::Config::WordLength::BITS_8;

        // user settings
        uart_config.periph        = config.periph;
        uart_config.pin_config.rx = config.rx;
        uart_config.pin_config.tx = config.tx;

        rx_buffer      = config.rx_buffer;
        rx_buffer_size = config.rx_buffer_size;

        /** zero the buffer to ensure emptiness regardless of source memory */
        std::fill(rx_buffer, rx_buffer + rx_buffer_size, 0);

        uart_.Init(uart_config);
    }

    /** @brief Start the UART peripheral in listening mode.
     *  This will fill an internal data structure in the background */
    inline void StartRx(SerRxParseCallback parse_callback, void* context)
    {
        parse_context_  = context;
        parse_callback_ = parse_callback;
        uvs_dma_clear_cache_for_buffer((uint8_t*)this,
                                       sizeof(SerialReceiver));
        uart_.DmaListenStart(
            rx_buffer, rx_buffer_size, SerialReceiver::rxCallback, this);
    }

    /** @brief returns whether the UART peripheral is actively listening in the background or not */
    inline bool RxActive() const { return uart_.IsListening(); }

    /** @brief This is a no-op for UART transport - Rx is via DMA callback with circular buffer */
    inline void FlushRx() {}

    /** @brief Returns UART HAL UART Error Code */
    inline uint32_t GetError() { return uart_.CheckError();}

    /** @brief sends the buffer of bytes out of the UART peripheral */
    inline void Tx(uint8_t* buff, size_t size) { uart_.PollTx(buff, size); }

  private:
    UartHandler        uart_;
    uint8_t*           rx_buffer;
    size_t             rx_buffer_size;
    void*              parse_context_;
    SerRxParseCallback parse_callback_;

    /** Static callback for Uart IBUS that occurs when
     *  new data is available from the peripheral.
     *  The new data is transferred from the peripheral to the
     *  IBUS instance's byte FIFO that feeds the IBUS parser.
     *
     *  TODO: Handle UartHandler errors better/at all.
     *  (If there is a UART error, there's not really any recovery
     *  option at the moment)
     */
    static void rxCallback(uint8_t*            data,
                           size_t              size,
                           void*               context,
                           UartHandler::Result res)
    {
        /** Read context as transport type */
        SerialReceiver* transport
            = reinterpret_cast<SerialReceiver*>(context);
        if(res == UartHandler::Result::OK)
        {
            if(transport->parse_callback_)
            {
                transport->parse_callback_(
                    data, size, transport->parse_context_);
            }
        }
    }
};

} // namespace uvos
