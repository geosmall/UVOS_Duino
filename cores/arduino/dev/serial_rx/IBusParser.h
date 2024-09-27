#pragma once

#include "dev/ProtocolParser.h"

namespace uvos
{

/** @brief   Utility class for parsing raw byte streams into IBUS messages
 *  @details Implemented as a state machine designed to parse one byte at a time
 *  supports max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)

  Example set of bytes coming over the iBUS line for setting servos: 
    20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
  Explanation
    Protocol length: 20
    Command code: 40 
    Channel 0: DB 5  -> value 0x5DB
    Channel 1: DC 5  -> value 0x5Dc
    Channel 2: 54 5  -> value 0x554
    Channel 3: DC 5  -> value 0x5DC
    Channel 4: E8 3  -> value 0x3E8
    Channel 5: D0 7  -> value 0x7D0
    Channel 6: D2 5  -> value 0x5D2
    Channel 7: E8 3  -> value 0x3E8
    Channel 8: DC 5  -> value 0x5DC
    Channel 9: DC 5  -> value 0x5DC
    Channel 10: DC 5 -> value 0x5DC
    Channel 11: DC 5 -> value 0x5DC
    Channel 12: DC 5 -> value 0x5DC
    Channel 13: DC 5 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */

struct IBusFrame
{
    uint16_t           channels[14];
    uint16_t           checksum;
};

// Length of IBus data frame minus checksum
constexpr size_t IBUS_FRAME_LEN_MINUS_CHECKSUM = 30;

class IBusParser : public ProtocolParser {
public:
    IBusParser();
    ~IBusParser() override = default;

    // Implement the byte parsing logic for IBus
    bool ParseByte(uint8_t byte, ParsedMessage* msg) override;

    void ResetParser() override;

private:
    // Define state variables for IBus parsing
    enum ParserState {
        WaitingForHeader0,      // Start, looking for header byte
        ParserHasHeader0,       // Found first header byte
        ParserHasHeader1,       // Found second header byte
        ParserHasFrame,         // Have received a full frame
        ParserHasCheckSum0,     // Has first checksum byte
    };

    ParserState pstate_;
    uint32_t char_count_;       // Move char_count here
    uint16_t running_checksum_; // Move running_checksum here
    uint16_t frame_checksum_;   // Move frame_checksum here
    ParsedMessage temp_msg_;
};

} // namespace uvos
