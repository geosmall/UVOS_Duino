#pragma once

#include <atomic>
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

class ProtocolParser {
public:
    ProtocolParser() : active_buffer_index_(0), message_ready_(false) {
        // Initialize the parsed messages
        messages_[0] = ParsedMessage();
        messages_[1] = ParsedMessage();
    }

    virtual ~ProtocolParser() = default;

    // Process a single byte. Return true if a complete message is parsed.
    virtual bool ParseByte(uint8_t byte, ParsedMessage* msg) = 0;

    // Reset the parser state
    virtual void ResetParser() = 0;

    // Listener function to be called from main loop to check for new messages
    bool Listener() const {
        return message_ready_.load(std::memory_order_acquire);
    }

    // Get the ready message if available
    bool GetMessage(ParsedMessage& msg) {
        if (message_ready_.load(std::memory_order_acquire)) {
            size_t ready_index = (active_buffer_index_.load(std::memory_order_acquire) == 0) ? 1 : 0;
            msg = messages_[ready_index];
            message_ready_.store(false, std::memory_order_release);
            return true;
        }
        return false;
    }

protected:
    // Invoke this callback from Parser subclass when a message is successfully parsed
    void NotifyParse(const ParsedMessage& msg) {
        message_ready_.store(true, std::memory_order_release);
        SwitchBuffers();
    }

    // Switch between active and ready buffers
    inline void SwitchBuffers() {
        // Switch to the other buffer by toggling the index
        active_buffer_index_.fetch_xor(1, std::memory_order_release);
    }

private:
    ParsedMessage messages_[2]; // Two buffers for parsed messages
    std::atomic<size_t> active_buffer_index_; // Index of the active buffer
    std::atomic<bool> message_ready_; // Flag to indicate if a new message is ready
};

} // namespace uvos
