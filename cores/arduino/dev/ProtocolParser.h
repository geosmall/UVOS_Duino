#pragma once

#include <cstdint>
#include <functional>

namespace uvos
{

// Forward declaration for parsed message structure
struct ParsedMessage {
    // Define fields common to all protocols or use a variant/union if necessary
    // Example:
    // enum class ProtocolType { IBus, SBus, ... };
    // ProtocolType protocol;
    // std::vector<uint8_t> data;
};

// Type alias for the parse callback
using SerRxParseCallback = std::function<void(const ParsedMessage&)>;

class ProtocolParser {
public:
    virtual ~ProtocolParser() = default;

    // Process a single byte. Return true if a complete message is parsed.
    virtual bool parse_byte(uint8_t byte) = 0;

    // Set the callback to notify when a message is parsed
    void set_parse_callback(SerRxParseCallback callback) {
        parse_callback_ = callback;
    }

protected:
    // Invoke this when a message is successfully parsed
    void notify_parse(const ParsedMessage& msg) {
        if (parse_callback_) {
            parse_callback_(msg);
        }
    }

private:
    SerRxParseCallback parse_callback_;
};

} // namespace uvos
