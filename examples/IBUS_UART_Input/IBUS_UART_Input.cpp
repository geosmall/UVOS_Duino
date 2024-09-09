#include "uvos_brd.h"

/** This prevents us from having to type "uvos::" in front of a lot of things. */
using namespace uvos;

/** Global Hardware access */
UVOSboard         hw;
IBusRxHandler     ibus_rx;

// Typical Switch case for Message Type.
void HandleIBusMessage(SerRxEvent event)
{
    uint16_t ch1;

    switch(event.type)
    {
        case RxValidPacket:
        {
            RxValidPacketEvent valid_packet = event.AsRxValidPacket();
            ch1 = valid_packet.channels[0];
        }
        break;
        case RxFailSafePacket:
        {
            RxFailSafePacketEvent fail_packet = event.AsRxFailSafePacket();
            ch1 = fail_packet.channels[0];
        }
        default: break;
    }
}

int main(void)
{
    /** Initialize our hardware */
    hw.Init();

    System::Delay(50);

    hw.StartLog();

    IBusRxHandler::Config ser_rx_config;
    ibus_rx.Init(ser_rx_config);

    ibus_rx.StartReceive();

    // uint32_t now = System::GetNow();

    /** Infinite Loop */
    while(1)
    {
        // now = System::GetNow();

        /** Process Serial Rx in the background */
        ibus_rx.Listen();

        // hw.PrintLine("---");
        // Handle IBus Events
        while(ibus_rx.HasEvents())
        {
            HandleIBusMessage(ibus_rx.PopEvent());
        }

        // Wait 500ms
        System::Delay(100);
    }
}