#include "uvos_brd.h"
#include "dev/serial_rx.h"
#include "dev/protocol_parser.h"
#include <cstring>

/** This prevents us from having to type "uvos::" in front of a lot of things. */
using namespace uvos;

/** Global Hardware access */
UVOSboard         hw;

uint8_t sumbuff[1024];

void UsbCallback(uint8_t* buf, uint32_t* len)
{
    for(size_t i = 0; i < *len; i++)
    {
        sumbuff[i] = buf[i];
    }
}

int main(void)
{
    /** Initialize our hardware */
    hw.Init();

    System::Delay(50);

    hw.usb_handle.Init(UsbHandle::FS_INTERNAL);

    int  tick_cnt = 0;
    bool ledstate = false;
    char buff[512];
    sprintf(buff, "Received:\t%d\r\n", tick_cnt);

    hw.usb_handle.TransmitInternal((uint8_t*)buff, strlen(buff));

    System::Delay(500);

    hw.usb_handle.SetReceiveCallback(UsbCallback, UsbHandle::FS_INTERNAL);

    // Create SerialReceiver of type IBUS, provide a parse = TRUE callback
    SerialReceiver ibus_rx(SerialReceiver::IBUS);

    // Config SerialReceiver UART and initialize it
    SerialReceiver::Config ser_rx_config;
    ibus_rx.Init(ser_rx_config);

    // Start the SerialReceiver
    ibus_rx.StartRx();

    ParsedMessage msg;

    /** Infinite Loop */
    while (1)
    {
        /** Process Serial Rx in the background */
        if (ibus_rx.Listener())
        {
            ibus_rx.GetMessage(&msg);
        }
        /** Print out the first 4 channels from the IBUS message
         * using fixed width formatting */
           sprintf(buff,
                "Ch1: %4d\tCh2: %4d\tCh3: %4d\tCh4: %4d\r\n",
                msg.channels[0],
                msg.channels[1],
                msg.channels[2],
                msg.channels[3]);
        hw.usb_handle.TransmitInternal((uint8_t*) buff, strlen(buff));

        // Wait 100ms
        System::Delay(100);
        hw.SetLed(ledstate = !ledstate);
    }
}