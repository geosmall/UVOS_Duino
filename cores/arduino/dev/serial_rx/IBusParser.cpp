#include "IBusParser.h"
#include <cstring>

namespace uvos
{

IBusParser::IBusParser()
{
    ResetParser();
}

bool IBusParser::ParseByte(uint8_t byte)
{
    bool did_parse = false;
    uint8_t i;

    switch (pstate_)
    {
    case WaitingForHeader0:
        // check byte for valid first header byte
        if (byte == 0x20)
        {
            byte_count_ = 1;
            running_checksum_ = 0xFFFF - byte;
            msg_.error_flags = 0;
            pstate_ = ParserHasHeader0; // we need to get the 2nd byte yet
        }
        break;
    case ParserHasHeader0:
        if (byte == 0x40)
        {
            byte_count_ = 2;
            running_checksum_ -= byte;
            pstate_ = ParserHasHeader1; // we need to get the 2nd byte yet
        }
        else
        {
            // invalid message go back to start
            ResetParser();
        }
        break;
    case ParserHasHeader1:
        byte_count_++;
        // Store the byte in the channel array
        // odd bytes are high byte, even bytes are low
        i = (byte_count_ - 3u) / 2u;
        if (i < SERRX_NUM_CHAN)
        {
            msg_.channels[i] = (byte_count_ % 2u) ? byte : (msg_.channels[i] | (byte << 8u));
        }
        running_checksum_ -= byte;
        if (byte_count_ >= IBUS_FRAME_LEN_MINUS_CHECKSUM)
        {
            pstate_ = ParserHasFrame;
        }
        break;
    case ParserHasFrame:
        frame_checksum_ = byte;
        pstate_ = ParserHasCheckSum0;
        break;
    case ParserHasCheckSum0:
        frame_checksum_ = (byte << 8) | frame_checksum_;
        if (frame_checksum_ == running_checksum_)
        {
            ParserNotify(); // good frame received
            did_parse = true;
        }
        // Go back to start
        ResetParser();
        break;
    default:
        break;
    }

    return did_parse;
}

void IBusParser::ResetParser()
{
    // Reset the parser state
    pstate_ = WaitingForHeader0;
    byte_count_ = 0;
    running_checksum_ = 0;
    frame_checksum_ = 0;

    std::memset(msg_.channels, 0, sizeof(msg_.channels)); // Zero out channels array
    msg_.error_flags = 0;
}

} // namespace uvos
