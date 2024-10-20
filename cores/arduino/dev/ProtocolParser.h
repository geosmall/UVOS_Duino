#pragma once

#include <cstdint>
#include <cstddef>
#include "util/FIFO.h"

namespace uvos
{
// Number of channels in the data frame
constexpr size_t SERRX_NUM_CHAN = 10;

// Forward declaration for parsed message structure
struct ParsedMessage {
    uint16_t channels[SERRX_NUM_CHAN]; // Channel data (1000-2000 us full range)
    uint32_t error_flags; // Error flags (bitmask)

    // Constructor to initialize the fields
    ParsedMessage() : channels{0}, error_flags(0) {}
};

// Abstract base class for protocol parsers
class ProtocolParser {
public:
    virtual ~ProtocolParser() = default;

    // Process a single byte. Return true if a complete message is parsed.
    // @param byte: The byte to parse
    // @return: True if a complete message is parsed
    virtual bool ParseByte(uint8_t byte) = 0;

    // Reset the parser state
    virtual void ResetParser() = 0;

    // Check if there are messages in the queue
    inline bool Listener() const
    {
        return !msg_q_.IsEmpty();
    }

    // Retrieve the next message from the queue
    inline bool GetMessageFromFIFO(ParsedMessage* msg)
    {
        if (msg == nullptr)
        {
            return false; // Handle null pointer case
        }
        return msg_q_.Get(*msg);
    }

protected:
    // Working message buffer
    ParsedMessage msg_;

    // Message queue
    uvos::FIFO<ParsedMessage, 16> msg_q_;

    // Invoke this callback from Parser subclass when a message is successfully parsed
    inline void ParserNotify()
    {
        msg_q_.PutWithOverwrite(msg_);
    }
};

} // namespace uvos
