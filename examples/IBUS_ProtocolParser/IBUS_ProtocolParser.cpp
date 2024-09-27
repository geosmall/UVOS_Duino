#include "uvos_brd.h"
#include "dev/serial_rx.h"
#include "dev/ProtocolParser.h"

/** This prevents us from having to type "uvos::" in front of a lot of things. */
using namespace uvos;

/** Global Hardware access */
UVOSboard         hw;
// IBusRxHandler     ibus_rx;
uint32_t time_stamp;

// Example callback function to handle parsed messages
void handle_parsed_message(const ParsedMessage& msg) {
    // Process the parsed message
    time_stamp = msg.timestamp;
 }

int main(void)
{
    /** Initialize our hardware */
    hw.Init();

    System::Delay(50);

    hw.StartLog();

    // Create SerialReceiver of type IBUS, provide a parse = TRUE callback
    SerialReceiver ibus_rx(SerialReceiver::IBUS, &handle_parsed_message);

    // Config SerialReceiver UART and initialize it
    SerialReceiver::Config ser_rx_config;
    ibus_rx.Init(ser_rx_config);

    // Start the SerialReceiver
    ibus_rx.StartRx();

    // uint32_t now = System::GetNow();

    /** Infinite Loop */
    while(1)
    {
        // now = System::GetNow();

        /** Process Serial Rx in the background */
        // ibus_rx.Listen();

        // Wait 500ms
        System::Delay(100);
    }
}