// Lock-Free Single-Producer-Single-Consumer (SPSC) Queue
//
// Purpose: Wait-free ring buffer for sensor data transfer between threads
// Based on: Dmitry Vyukov's SPSC queue algorithm (https://www.1024cores.net/home/lock-free-algorithms/queues/bounded-mpmc-queue)
//
// Key Features:
// - Zero-allocation after construction
// - Wait-free enqueue/dequeue (O(1) guaranteed)
// - Cache-aligned to prevent false sharing
// - Supports arbitrary POD types
//
// Sample Input:
//   LockFreeQueue<ImuSample> queue(512);
//   ImuSample sample = {timestamp, gyro, accel};
//   queue.push(sample);
//
// Expected Output:
//   ImuSample retrieved;
//   bool success = queue.pop(retrieved);  // true if data available
//
// Thread Safety:
//   - ONE producer thread calls push()
//   - ONE consumer thread calls pop()
//   - NO synchronization needed between push/pop
//
// Performance Target: <100ns per operation on Snapdragon AR1 Gen 1

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>  // For std::hardware_destructive_interference_size
#include <stdexcept>
#include <type_traits>

namespace fusion {

// Cache line size for alignment (64 bytes on ARM/x86)
#ifdef __cpp_lib_hardware_interference_size
    constexpr size_t CACHE_LINE_SIZE = std::hardware_destructive_interference_size;
#else
    constexpr size_t CACHE_LINE_SIZE = 64;
#endif

template <typename T>
class LockFreeQueue {
    // Require copyable and trivially destructible (safe for lock-free operations)
    static_assert(std::is_copy_constructible_v<T> && std::is_copy_assignable_v<T>,
                  "LockFreeQueue requires copyable types");
    static_assert(std::is_trivially_destructible_v<T>,
                  "LockFreeQueue requires trivially destructible types");

public:
    explicit LockFreeQueue(size_t capacity)
        : capacity_(capacity),
          buffer_(new T[capacity_]),
          head_(0),
          tail_(0) {

        if (capacity == 0 || (capacity & (capacity - 1)) != 0) {
            // Capacity must be power of 2 for efficient modulo
            throw std::invalid_argument("Capacity must be power of 2");
        }
    }

    ~LockFreeQueue() = default;

    // Non-copyable, non-movable (contains atomics)
    LockFreeQueue(const LockFreeQueue&) = delete;
    LockFreeQueue& operator=(const LockFreeQueue&) = delete;
    LockFreeQueue(LockFreeQueue&&) = delete;
    LockFreeQueue& operator=(LockFreeQueue&&) = delete;

    // Producer: Enqueue element (returns false if full)
    bool push(const T& item) {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next_head = (head + 1) & mask();

        // Check if queue is full
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false;  // Queue full
        }

        // Write data
        buffer_[head] = item;

        // Publish update to head (release semantics for consumer)
        head_.store(next_head, std::memory_order_release);

        return true;
    }

    // Consumer: Dequeue element (returns false if empty)
    bool pop(T& item) {
        const size_t tail = tail_.load(std::memory_order_relaxed);

        // Check if queue is empty
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;  // Queue empty
        }

        // Read data
        item = buffer_[tail];

        // Publish update to tail (release semantics for producer)
        const size_t next_tail = (tail + 1) & mask();
        tail_.store(next_tail, std::memory_order_release);

        return true;
    }

    // Check if queue is empty (approximate, lock-free)
    bool empty() const {
        return tail_.load(std::memory_order_acquire) ==
               head_.load(std::memory_order_acquire);
    }

    // Get approximate size (may be stale)
    size_t size() const {
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & mask();
    }

    // Get capacity (actual usable slots is capacity - 1 due to full/empty distinction)
    size_t capacity() const {
        return capacity_ - 1;
    }

private:
    size_t mask() const {
        return capacity_ - 1;
    }

    const size_t capacity_;
    std::unique_ptr<T[]> buffer_;

    // Cache-line aligned atomics to prevent false sharing
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> head_;  // Producer writes
    alignas(CACHE_LINE_SIZE) std::atomic<size_t> tail_;  // Consumer writes
};

}  // namespace fusion
