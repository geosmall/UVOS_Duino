#include "IBusParser.h"

namespace uvos
{

IBusParser::IBusParser()
{
    ResetParser();
}

bool IBusParser::ParseByte(uint8_t byte, ParsedMessage* pMsg)
{
    bool did_parse = false;
    uint8_t i;

    switch (pstate_)
    {
    case WaitingForHeader0:
        // check byte for valid first header byte
        if (byte == 0x20)
        {
            char_count_ = 1;
            running_checksum_ = 0xFFFF - byte;
            pstate_ = ParserHasHeader0; // we need to get the 2nd byte yet
        }
        break;
    case ParserHasHeader0:
        if (byte == 0x40)
        {
            char_count_ = 2;
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
        char_count_++;
        // Store the byte in the channel array
        // odd bytes are high byte, even bytes are low
        // channels array index is char_count_ / 2 - 1
        i = (char_count_ - 3u) / 2u;
        if ((i < SERRX_NUM_CHAN) && (pMsg != nullptr))
        {
            if (char_count_ % 2u)
            {
                pMsg->channels[i] = byte; // odd = low byte
            }
            else
            {
                pMsg->channels[i] |= byte << 8u; // even = high bytr
            }
        }
        running_checksum_ -= byte;
        if (char_count_ >= IBUS_FRAME_LEN_MINUS_CHECKSUM)
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
            if (pMsg != nullptr)
            {
                pMsg->error_flags = 0;
                ParserNotify(pMsg);
            }
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
    char_count_ = 0;
    running_checksum_ = 0;
    frame_checksum_ = 0;
}

} // namespace uvos
