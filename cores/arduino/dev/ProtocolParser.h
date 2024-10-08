#pragma once

#include <cstdint>
#include <cstddef>

namespace uvos
{
// Number of channels in the data frame
constexpr size_t SERRX_NUM_CHAN = 10;

// Forward declaration for parsed message structure
struct ParsedMessage {
    uint32_t timestamp; // Timestamp of frame in microseconds
    uint16_t channels[SERRX_NUM_CHAN]; // Channel data (1000-2000 us full range) 
    uint32_t error_flags; // Error flags (bitmask)

    // Constructor to initialize the fields, including timestamp and channels
    ParsedMessage() : timestamp(0), channels{0}, error_flags(0) {}
};

// Type alias for the parse callback, called when a message is parsed
// using SerRxParseCallback = std::function<void(const ParsedMessage&)>;
typedef void (*SerRxParseCallback)(const ParsedMessage&);

class ProtocolParser {
public:
    virtual ~ProtocolParser() = default;

    // Process a single byte. Return true if a complete message is parsed.
    virtual bool ParseByte(uint8_t byte, ParsedMessage* msg) = 0;

    // Set the callback to notify when a message is parsed
    void SetParseCallback(SerRxParseCallback callback) {
        parse_callback_ = callback;
    }

    // Reset the parser state
    virtual void ResetParser() = 0;

protected:
    // Invoke this when a message is successfully parsed
    void NotifyParse(const ParsedMessage& msg) {
        if (parse_callback_) {
            parse_callback_(msg);
        }
    }

private:
    SerRxParseCallback parse_callback_;
};

} // namespace uvos
