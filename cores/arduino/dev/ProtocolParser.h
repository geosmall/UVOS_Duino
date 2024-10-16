#pragma once

#include <cstdint>
#include <cstddef>
#include "util/FIFO.h"
#include "sys/system.h"

namespace uvos
{
// Number of channels in the data frame
constexpr size_t SERRX_NUM_CHAN = 10;

// Forward declaration for parsed message structure
struct ParsedMessage {
    uint32_t timestamp; // Timestamp of frame in milliseconds
    uint16_t channels[SERRX_NUM_CHAN]; // Channel data (1000-2000 us full range)
    uint32_t error_flags; // Error flags (bitmask)

    // Constructor to initialize the fields
    ParsedMessage() : timestamp(0), channels{0}, error_flags(0) {}
};

// Abstract base class for protocol parsers
class ProtocolParser {
public:
    virtual ~ProtocolParser() = default;

    // Process a single byte. Return true if a complete message is parsed.
    // @param byte: The byte to parse
    // @param pMsg: Pointer to the parsed message structure
    // @return: True if a complete message is parsed
    virtual bool ParseByte(uint8_t byte, ParsedMessage* pMsg) = 0;

    // Reset the parser state
    virtual void ResetParser() = 0;

    // Check if there are messages in the queue
    inline bool Listener() const
    {
        return !msg_q_.IsEmpty();
    }

    // Retrieve the next message from the queue
    inline bool GetMessage(ParsedMessage& msg)
    {
        return msg_q_.Get(msg);
    }

protected:
    // Message queue
    uvos::FIFO<ParsedMessage, 16> msg_q_;

    // Invoke this callback from Parser subclass when a message is successfully parsed
    inline void ParserNotify(const ParsedMessage* pMsg)
    {
        if (pMsg != nullptr)
        {
            ParsedMessage msg = *pMsg; // Dereference the message pointer
            msg.timestamp = System::GetNow(); // Give it a timestamp
            msg_q_.PutWithOverwrite(msg); // Put the message in the queue
        }
    }
};

} // namespace uvos
