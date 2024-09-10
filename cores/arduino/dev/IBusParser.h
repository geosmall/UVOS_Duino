#pragma once

#include "ProtocolParser.h"

namespace uvos
{

class IBusParser : public ProtocolParser {
public:
    IBusParser();
    ~IBusParser() override = default;

    // Implement the byte parsing logic for IBus
    bool parse_byte(uint8_t byte) override;

private:
    // Define state variables for IBus parsing
    enum class State {
        WaitingForHeader,
        ReceivingData,
        // ... other states
    };

    State current_state_;
    // Additional members like buffer, counters, etc.
};

} // namespace uvos
