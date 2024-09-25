#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

// Number of channels in the data frame
constexpr size_t NUM_CHANNELS = 10;

namespace uvos
{

// Forward declaration for parsed message structure
struct ParsedMessage {
    uint32_t timestamp; // Timestamp of frame in microseconds
    std::vector<uint16_t> channels; // Channel data (1000-2000 us full range) 
    uint32_t error_flags; // Error flags (bitmask)

    // Constructor to initialize the fields, including timestamp and channels
    ParsedMessage() : timestamp(0), channels(NUM_CHANNELS, 0) {}
};

// Type alias for the parse callback, called when a message is parsed
// using SerRxParseCallback = std::function<void(const ParsedMessage&)>;
typedef void (*SerRxParseCallback)(const ParsedMessage&);

class ProtocolParser {
public:
    virtual ~ProtocolParser() = default;

    // Process a single byte. Return true if a complete message is parsed.
    virtual bool parse_byte(uint8_t byte, ParsedMessage* msg) = 0;

    // Set the callback to notify when a message is parsed
    void set_parse_callback(SerRxParseCallback callback) {
        parse_callback_ = callback;
    }

    // Reset the parser state
    virtual void reset() = 0;

private:
    SerRxParseCallback parse_callback_;
};

} // namespace uvos
