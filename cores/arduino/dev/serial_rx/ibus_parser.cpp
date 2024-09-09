#include "ibus_parser.h"

using namespace uvos;

bool IBusParser::Parse(uint8_t byte, SerRxEvent* event_out)
{
    bool did_parse = false;
    uint8_t i;

    switch (pstate_)
    {
    case ParserEmpty:
        // check byte for valid first header byte
        if (byte == 0x20)
        {
            char_count_ = 1;
            running_checksum_ = 0xFFFF - byte;
            pstate_ = ParserHaveHeader0; // we need to get the 2nd byte yet
        }
        break;
    case ParserHaveHeader0:
        if (byte == 0x40)
        {
            char_count_ = 2;
            running_checksum_ -= byte;
            pstate_ = ParserHaveHeader1; // we need to get the 2nd byte yet
        }
        else
        {
            // invalid message go back to start
            char_count_ = 0;
            pstate_ = ParserEmpty;
        }
        break;
    case ParserHaveHeader1:
        char_count_++;
        // Store the byte in the channel array
        // odd bytes are high byte, even bytes are low
        // channels array index is char_count_ / 2 - 1
        i = (char_count_ - 3u) / 2u;
        if (i < SERRX_NUM_CHAN)
        {
            if (char_count_ % 2u)
            {
                event_message_.channels[i] = byte; // odd = low byte
            }
            else
            {
                event_message_.channels[i] |= byte << 8u; // even = high bytr
            }
        }
        running_checksum_ -= byte;
        if (char_count_ >= 30u)
        {
            pstate_ = ParserHaveFrame;
        }
        break;
    case ParserHaveFrame:
        frame_checksum_ = byte;
        pstate_ = ParserHaveCheckSum0;
        break;
    case ParserHaveCheckSum0:
        frame_checksum_ = (byte << 8) | frame_checksum_;
        if (frame_checksum_ == running_checksum_)
        {
            event_message_.type = RxValidPacket;

            if (event_out != nullptr)
            {
                *event_out = event_message_;
            }
            did_parse = true;
        }
        // Go back to start
        char_count_ = 0;
        pstate_ = ParserEmpty;
        break;
    default:
        break;
    }

    return did_parse;
}

void IBusParser::Reset()
{
    pstate_ = ParserEmpty;
    event_message_.type = RxMessageLast;
}
