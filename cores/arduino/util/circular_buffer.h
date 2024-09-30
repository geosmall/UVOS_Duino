#pragma once

#include <array>
#include <atomic>
#include <stm32yyxx.h>

namespace uvos
{

// Utility to check if a number is a power of two
constexpr bool is_power_of_two(size_t n)
{
    return (n != 0) && ((n & (n - 1)) == 0);
}

template <class T, size_t TElemCount> class circular_buffer
{
    // Static assertion to enforce that TElemCount is a power of two
    static_assert(is_power_of_two(TElemCount), "TElemCount must be a power of two");

  public:
    explicit circular_buffer() = default;

    // Put an item into the circular buffer, overwriting the oldest item if full
    void put(T item) noexcept
    {
        buf_[head_.load(std::memory_order_relaxed)] = item;

        if (full_.load(std::memory_order_relaxed))
        {
            // Advance the tail if the buffer is full
            tail_.store((tail_.load(std::memory_order_relaxed) + 1) & (TElemCount - 1), std::memory_order_relaxed);
        }

        // Advance the head
        head_.store((head_.load(std::memory_order_relaxed) + 1) & (TElemCount - 1), std::memory_order_release);

        // Set the buffer as full if head and tail collide
        full_.store(head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed), std::memory_order_release);
    }

    // Get an item from the circular buffer
    T get() noexcept
    {
        // Check if the buffer is empty
        if (empty())
        {
            return T();
        }

        // Retrieve the item from the buffer and advance the tail
        auto val = buf_[tail_.load(std::memory_order_relaxed)];
        full_.store(false, std::memory_order_release);
        tail_.store((tail_.load(std::memory_order_relaxed) + 1) & (TElemCount - 1), std::memory_order_release);

        return val;
    }

    // Reset the buffer to an empty state
    void reset() noexcept
    {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        full_.store(false, std::memory_order_relaxed);
    }

    // Check if the buffer is empty
    bool empty() const noexcept
    {
        return (!full_.load(std::memory_order_acquire) && (head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire)));
    }

    // Check if the buffer is full
    bool full() const noexcept
    {
        return full_.load(std::memory_order_acquire);
    }

    // Get the capacity of the buffer
    size_t capacity() const noexcept
    {
        return TElemCount;
    }

    // Get the current size of the buffer
    size_t size() const noexcept
    {
        size_t size = TElemCount;

        if (!full_.load(std::memory_order_acquire))
        {
            size_t head = head_.load(std::memory_order_acquire);
            size_t tail = tail_.load(std::memory_order_acquire);

            if (head >= tail)
            {
                size = head - tail;
            }
            else
            {
                size = TElemCount + head - tail;
            }
        }

        return size;
    }

  private:
    std::array<T, TElemCount> buf_ = {}; // Buffer storage
    std::atomic<size_t> head_ = 0;       // Head index
    std::atomic<size_t> tail_ = 0;       // Tail index
    std::atomic<bool> full_ = false;     // Buffer full status
};

} // namespace uvos
