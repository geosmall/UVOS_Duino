#pragma once

namespace uvos
{

// Spinlock implementation using ARM Cortex-M7 LDREX/STREX instructions
class spinlock
{
  public:
    void lock()
    {
        while (true)
        {
            // Load the lock state with exclusive access
            uint32_t status = __LDREXW(&lock_);

            // If the lock is not taken (status == 0), try to acquire it
            if (status == 0)
            {
                // Attempt to set the lock to 1
                if (__STREXW(1, &lock_) == 0)
                {
                    // Successfully acquired the lock
                    __DMB(); // Data Memory Barrier to ensure proper memory ordering
                    return;
                }
            }

            // Retry until the lock is acquired
        }
    }

    void unlock()
    {
        // Ensure memory operations are completed before releasing the lock
        __DMB();
        // Release the lock by setting it to 0
        lock_ = 0;
    }

  private:
    volatile uint32_t lock_ = 0;
};

} // namespace uvos
