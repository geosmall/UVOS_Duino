# SPSC Lock free ring buffer discussion

## Framing

### Question:

Please analyze this ring buffer c++ implementation for appropriate use of atomic operations including std::memory_order specifiers for use in multi-threaded spsc context..

NOTE - The buffer is intended to be used in spsc interupt handler context on stm32h743 cortex-m7.

```C++

/**************************************************************
 * @file ring_buf.h
 * @brief A ring buffer implementation written in standard
 * c++11 suitable for both low-end microcontrollers. Lock-free
 * for single consumer single producer scenarios.
 *
 * This file based on lockfree ring_buf.hpp
 * See https://github.com/DNedic/lockfree
 * Removed <span> interface and consolidated implm
 *
 * Author:          Djordje Nedic <nedic.djordje2@gmail.com>
 * Version:         v2.0.9
 **************************************************************/

/************************** INCLUDE ***************************/
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace uvos
{

/*************************** TYPES ****************************/

template <typename T, size_t size> class RingBuf {
    static_assert(std::is_trivial<T>::value, "The type T must be trivial");
    static_assert(size > 2, "Buffer size must be bigger than 2");

    /********************** PUBLIC METHODS ************************/
  public:
    RingBuf();

    /**
     * @brief Writes data to the ring buffer.
     * Should only be called from the producer thread.
     * @param[in] Pointer to the data to write
     * @param[in] Number of elements to write
     * @retval Write success
     */
    bool Write(const T *data, size_t cnt);

    /**
     * @brief Writes data to the ring buffer.
     * Should only be called from the producer thread.
     * @param[in] Data array to write
     * @retval Write success
     */
    template <size_t arr_size> bool Write(const std::array<T, arr_size> &data);

    /**
     * @brief Reads data from the ring buffer.
     * Should only be called from the consumer thread.
     * @param[out] Pointer to the space to read the data to
     * @param[in] Number of elements to read
     * @retval Write success
     */
    bool Read(T *data, size_t cnt);

    /**
     * @brief Reads data from the ring buffer.
     * Should only be called from the consumer thread.
     * @param[out] Array to write the read to
     * @retval Write success
     */
    template <size_t arr_size> bool Read(std::array<T, arr_size> &data);

    /**
     * @brief Reads data from the ring buffer without consuming it, meant to
     * be used in conjunction with Skip().
     * The combination is most useful when we want to keep the data in the
     * buffer after some operation using the data fails, or uses only some of
     * it. Should only be called from the consumer thread.
     * @param[out] Pointer to the space to read the data to
     * @param[in] Number of elements to read
     * @retval Write success
     */
    bool Peek(T *data, size_t cnt) const;

    /**
     * @brief Reads data from the ring buffer without consuming it, meant to
     * be used in conjunction with Skip().
     * The combination is most useful when we want to keep the data in the
     * buffer after some operation using the data fails, or uses only some of
     * it. Should only be called from the consumer thread.
     * @param[out] Array to write the read to
     * @retval Write success
     */
    template <size_t arr_size> bool Peek(std::array<T, arr_size> &data) const;

    /**
     * @brief Makes the ring buffer skip the oldest data, meant to be used
     * in conjunction with Peek().
     * The combination is most useful when we want to keep the data in the
     * buffer after some operation using the data fails, or uses only some of
     * it. Should only be called from the consumer thread.
     * @param[in] Number of elements to skip
     * @retval Write success
     */
    bool Skip(size_t cnt);

    /**
     * @brief Gets the number of free slots in the ring buffer.
     * Just like Write(), this should be called only from the producer thread.
     * @retval Number of free slots
     */
    size_t GetFree() const;

    /**
     * @brief Gets the number of available elements in the ring buffer.
     * Just like Read(), this should be called only from the consumer thread.
     * @retval Number of available elements
     */
    size_t GetAvailable() const;

    /********************* PRIVATE METHODS ************************/
  private:
    static size_t CalcFree(const size_t w, const size_t r);
    static size_t CalcAvailable(const size_t w, const size_t r);

    /********************** PRIVATE MEMBERS ***********************/
  private:
    T _data[size]; /**< Data array */
    std::atomic_size_t _r; /**< Read index */
    std::atomic_size_t _w; /**< Write index */
};

/********************** PUBLIC METHODS ************************/

template <typename T, size_t size>
RingBuf<T, size>::RingBuf() : _r(0U), _w(0U) {}

template <typename T, size_t size>
bool RingBuf<T, size>::Write(const T *data, const size_t cnt) {
    /* Preload variables with adequate memory ordering */
    size_t w = _w.load(std::memory_order_relaxed);
    const size_t r = _r.load(std::memory_order_acquire);

    if (CalcFree(w, r) < cnt) {
        return false;
    }

    /* Check if the write wraps */
    if (w + cnt <= size) {
        /* Copy in the linear region */
        memcpy(&_data[w], &data[0], cnt * sizeof(T));
        /* Correct the write index */
        w += cnt;
        if (w == size) {
            w = 0U;
        }
    } else {
        /* Copy in the linear region */
        const size_t linear_free = size - w;
        memcpy(&_data[w], &data[0], linear_free * sizeof(T));
        /* Copy remaining to the beginning of the buffer */
        const size_t remaining = cnt - linear_free;
        memcpy(&_data[0], &data[linear_free], remaining * sizeof(T));
        /* Correct the write index */
        w = remaining;
    }

    /* Store the write index with adequate ordering */
    _w.store(w, std::memory_order_release);

    return true;
}

template <typename T, size_t size>
bool RingBuf<T, size>::Read(T *data, const size_t cnt) {
    /* Preload variables with adequate memory ordering */
    size_t r = _r.load(std::memory_order_relaxed);
    const size_t w = _w.load(std::memory_order_acquire);

    if (CalcAvailable(w, r) < cnt) {
        return false;
    }

    /* Check if the read wraps */
    if (r + cnt <= size) {
        /* Copy in the linear region */
        memcpy(&data[0], &_data[r], cnt * sizeof(T));
        /* Correct the read index */
        r += cnt;
        if (r == size) {
            r = 0U;
        }
    } else {
        /* Copy in the linear region */
        const size_t linear_available = size - r;
        memcpy(&data[0], &_data[r], linear_available * sizeof(T));
        /* Copy remaining from the beginning of the buffer */
        const size_t remaining = cnt - linear_available;
        memcpy(&data[linear_available], &_data[0], remaining * sizeof(T));
        /* Correct the read index */
        r = remaining;
    }

    /* Store the write index with adequate ordering */
    _r.store(r, std::memory_order_release);

    return true;
}

template <typename T, size_t size>
bool RingBuf<T, size>::Peek(T *data, const size_t cnt) const {
    /* Preload variables with adequate memory ordering */
    const size_t r = _r.load(std::memory_order_relaxed);
    const size_t w = _w.load(std::memory_order_acquire);

    if (CalcAvailable(w, r) < cnt) {
        return false;
    }

    /* Check if the read wraps */
    if (r + cnt <= size) {
        /* Copy in the linear region */
        memcpy(&data[0], &_data[r], cnt * sizeof(T));
    } else {
        /* Copy in the linear region */
        const size_t linear_available = size - r;
        memcpy(&data[0], &_data[r], linear_available * sizeof(T));
        /* Copy remaining from the beginning of the buffer */
        const size_t remaining = cnt - linear_available;
        memcpy(&data[linear_available], &_data[0], remaining * sizeof(T));
    }

    return true;
}

template <typename T, size_t size>
bool RingBuf<T, size>::Skip(const size_t cnt) {
    /* Preload variables with adequate memory ordering */
    size_t r = _r.load(std::memory_order_relaxed);
    const size_t w = _w.load(std::memory_order_acquire);

    if (CalcAvailable(w, r) < cnt) {
        return false;
    }

    r += cnt;
    /* Wrap the index if necessary */
    if (r >= size) {
        r -= size;
    }

    /* Store the write index with adequate ordering */
    _r.store(r, std::memory_order_release);

    return true;
}

template <typename T, size_t size> size_t RingBuf<T, size>::GetFree() const {
    const size_t w = _w.load(std::memory_order_relaxed);
    const size_t r = _r.load(std::memory_order_acquire);

    return CalcFree(w, r);
}

template <typename T, size_t size>
size_t RingBuf<T, size>::GetAvailable() const {
    const size_t r = _r.load(std::memory_order_relaxed);
    const size_t w = _w.load(std::memory_order_acquire);

    return CalcAvailable(w, r);
}

/********************** std::array API ************************/

template <typename T, size_t size>
template <size_t arr_size>
bool RingBuf<T, size>::Write(const std::array<T, arr_size> &data) {
    return Write(data.begin(), arr_size);
}

template <typename T, size_t size>
template <size_t arr_size>
bool RingBuf<T, size>::Read(std::array<T, arr_size> &data) {
    return Read(data.begin(), arr_size);
}

template <typename T, size_t size>
template <size_t arr_size>
bool RingBuf<T, size>::Peek(std::array<T, arr_size> &data) const {
    return Peek(data.begin(), arr_size);
}

/********************* PRIVATE METHODS ************************/

template <typename T, size_t size>
size_t RingBuf<T, size>::CalcFree(const size_t w, const size_t r) {
    if (r > w) {
        return (r - w) - 1U;
    } else {
        return (size - (w - r)) - 1U;
    }
}

template <typename T, size_t size>
size_t RingBuf<T, size>::CalcAvailable(const size_t w, const size_t r) {
    if (w >= r) {
        return w - r;
    } else {
        return size - (r - w);
    }
}

} // namespace uvos

````

### ChatGPT response:

The ring buffer implementation is designed for a single-producer, single-consumer (SPSC) scenario, which simplifies the need for complex synchronization mechanisms. However, for safe use in a multi-threaded environment, especially in the interrupt handler context on the STM32H743 Cortex-M7, the use of atomic operations and memory ordering is crucial. Here's an analysis of your current implementation with a focus on the appropriate use of atomic operations and memory orderings:

### Key Points:

1. **Atomic Variables**:
   - You use `std::atomic_size_t` for both the read (`_r`) and write (`_w`) indices, which is essential for ensuring correct visibility and atomicity of operations across the producer and consumer contexts.
   
2. **Memory Ordering**:
   - **Relaxed Operations** (`std::memory_order_relaxed`):
     - These are used in places where synchronization between threads is not strictly necessary for the current operation (e.g., when loading indices without needing visibility guarantees). In the `Write()` and `Read()` methods, `std::memory_order_relaxed` is used to load `_r` (read index) and `_w` (write index) for local computation. This is appropriate because these operations don't immediately need synchronization with the other thread.
   
   - **Acquire-Release Semantics**:
     - **Acquire (`std::memory_order_acquire`)**: Used when loading `_r` and `_w` in both `Write()` and `Read()`. This ensures that any prior stores (from the other thread) are visible before reading these values. This is crucial to ensure data consistency before accessing the buffer contents.
     - **Release (`std::memory_order_release`)**: Used when storing the updated values of `_w` and `_r` after write or read operations. This ensures that all previous writes to the buffer (by the producer) or reads (by the consumer) are visible before the index update becomes visible to the other thread.
   
   - **Potential Optimizations**:
     - In both `Write()` and `Read()` operations, you store the index with `std::memory_order_release`, which is appropriate. However, when reading the other index (e.g., the consumer reading the producer’s `_w`), `std::memory_order_acquire` is also used correctly to ensure that the latest updates are visible.
   
3. **Handling Wraparound**:
   - The implementation handles the wraparound condition when the indices (`_r` and `_w`) reach the buffer's end, ensuring correctness in both linear and wrapped regions. This is essential for the integrity of the buffer in lock-free SPSC usage.

4. **Interrupt Context**:
   - Since this buffer may be used in an interrupt handler context (common in embedded systems), the operations must remain lock-free and atomic to prevent race conditions. Your use of atomic variables with proper memory ordering ensures that operations on `_r` and `_w` are both atomic and correctly synchronized between producer and consumer. This design ensures that the buffer will behave correctly in an ISR context.

### Improvements and Considerations:

1. **Cache Coherency**:
   - On the STM32H743 Cortex-M7, DMA and interrupt-based operations often need to manage cache coherency manually (especially if the buffer is placed in a region affected by cache). Ensure that you are handling the cache (e.g., using `SCB_InvalidateDCache_by_Addr()` or similar) if the buffer is accessed via DMA or located in cacheable memory.

2. **Processor Intrinsics**:
   - If performance is critical, and if you're not constrained to standard C++, you could explore using processor-specific intrinsics (`LDREX`/`STREX` on ARM) for atomics in a low-level context. However, given that `std::atomic` is generally mapped to these intrinsics, your current implementation with C++ atomics should be efficient for this use case.

### Conclusion:
Your implementation makes appropriate use of atomic operations and memory orderings for an SPSC context. The combination of `memory_order_relaxed` for non-critical loads and `memory_order_acquire/release` for critical synchronization points is optimal for ensuring correctness without unnecessary overhead. This design should be suitable for interrupt handler use on the STM32H743, assuming proper cache management when needed.

## Summary

### Question:
Please summarize the list of assumptions that must be true for ring buffer to be lock free in its intended uses in multi-treaded spsc (interrupt handler producer and user thread consumer) context on stm32h743 cortex-m7 while maintaing data integrity

### ChatGPT response:
For the ring buffer to remain lock-free and maintain data integrity in a single-producer, single-consumer (SPSC) context on the STM32H743 Cortex-M7, where the producer operates in an interrupt handler and the consumer in a user thread, the following assumptions must be true:

Single Producer, Single Consumer:

- Only one producer (interrupt handler) and one consumer (user thread) are interacting with the buffer. No multiple producers or consumers are allowed.
Atomicity of Index Operations:

- The read (_r) and write (_w) indices are updated atomically using std::atomic_size_t, ensuring that index modifications are atomic and visible between threads without race conditions.

Correct Memory Ordering (Memory operations are ordered correctly):

- The producer uses std::memory_order_release to store the write index, ensuring all writes to the buffer happen before the updated index is visible to the consumer.
- The consumer uses std::memory_order_acquire when reading the write index, ensuring that it sees the most up-to-date data.
- Similarly, the consumer uses std::memory_order_release to store the read index, and the producer uses std::memory_order_acquire to read the read index to ensure synchronization of read operations.

No Overflow of Write Index:
- The producer checks that there is enough free space in the buffer before writing. This ensures that the write index does not overwrite unread data, which could lead to data corruption.

Wraparound Handling:
- Both the read and write indices are wrapped around when they reach the buffer's end, ensuring the buffer functions correctly as a circular queue.

Proper Cache Management:

- Cache coherency is handled properly. On STM32H743 Cortex-M7, if the buffer resides in a cacheable memory region, appropriate cache invalidation and cleaning (e.g., using SCB_InvalidateDCache_by_Addr() and SCB_CleanDCache_by_Addr()) must be applied to ensure that both the producer and consumer see the latest data.

Buffer in Non-Cacheable Memory (optional but preferred):

- To avoid cache issues altogether, the buffer may be placed in a non-cacheable memory region (e.g., DTCM or tightly-coupled memory), especially if cache management overhead is not desirable.

Interrupts are Not Masked in Consumer:

- The consumer thread (user thread) should not mask interrupts for extended periods, as this could delay the producer's ability to update the buffer.

Non-blocking Operations:

- Both producer and consumer must only use non-blocking operations. If any operation on the buffer were to block, it would no longer be considered lock-free.
No Memory Fences Between Threads:

- No synchronization fences or locks are used between the interrupt handler and the consumer thread, relying solely on atomic operations and memory ordering to ensure correctness.

## Summary

If all these assumptions hold, the ring buffer can be used safely and lock-free in an SPSC context on the STM32H743 Cortex-M7.