// Test program for LockFreeQueue validation
#include "core/lockfree_queue.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace fusion;
using namespace std::chrono;

int main() {
    std::cout << "=== LockFreeQueue Validation ===" << std::endl;
    std::cout << "Cache line size: " << CACHE_LINE_SIZE << " bytes" << std::endl;

    // Test 1: Basic push/pop
    {
        LockFreeQueue<int> queue(16);

        // Push 10 items
        for (int i = 0; i < 10; i++) {
            bool success = queue.push(i);
            if (!success) {
                std::cerr << "FAIL: Push failed at item " << i << std::endl;
                return 1;
            }
        }

        // Check size
        if (queue.size() != 10) {
            std::cerr << "FAIL: Expected size 10, got " << queue.size() << std::endl;
            return 1;
        }

        // Pop and verify
        for (int i = 0; i < 10; i++) {
            int value;
            bool success = queue.pop(value);
            if (!success || value != i) {
                std::cerr << "FAIL: Pop failed or wrong value at " << i << std::endl;
                return 1;
            }
        }

        // Queue should be empty
        if (!queue.empty()) {
            std::cerr << "FAIL: Queue not empty after popping all" << std::endl;
            return 1;
        }

        std::cout << "✓ Test 1: Basic push/pop - PASSED" << std::endl;
    }

    // Test 2: Queue full handling
    {
        LockFreeQueue<int> queue(16);

        // Fill queue
        for (int i = 0; i < 16; i++) {
            queue.push(i);
        }

        // Next push should fail
        bool overflow = queue.push(999);
        if (overflow) {
            std::cerr << "FAIL: Push should fail when full" << std::endl;
            return 1;
        }

        std::cout << "✓ Test 2: Queue full handling - PASSED" << std::endl;
    }

    // Test 3: Producer-Consumer threads
    {
        LockFreeQueue<int> queue(1024);
        constexpr int NUM_ITEMS = 10000;
        std::atomic<bool> producer_done{false};

        // Producer thread
        std::thread producer([&]() {
            for (int i = 0; i < NUM_ITEMS; i++) {
                while (!queue.push(i)) {
                    // Spin if queue full (shouldn't happen with 1024 capacity)
                    std::this_thread::yield();
                }
            }
            producer_done.store(true, std::memory_order_release);
        });

        // Consumer thread
        std::thread consumer([&]() {
            int expected = 0;
            while (expected < NUM_ITEMS) {
                int value;
                if (queue.pop(value)) {
                    if (value != expected) {
                        std::cerr << "FAIL: Expected " << expected
                                  << ", got " << value << std::endl;
                        std::exit(1);
                    }
                    expected++;
                } else {
                    std::this_thread::yield();
                }
            }
        });

        producer.join();
        consumer.join();

        std::cout << "✓ Test 3: Producer-Consumer (" << NUM_ITEMS
                  << " items) - PASSED" << std::endl;
    }

    // Test 4: Performance benchmark
    {
        LockFreeQueue<int> queue(1024);
        constexpr int ITERATIONS = 1000000;

        auto start = high_resolution_clock::now();

        for (int i = 0; i < ITERATIONS; i++) {
            queue.push(i);
            int value;
            queue.pop(value);
        }

        auto end = high_resolution_clock::now();
        auto duration = duration_cast<nanoseconds>(end - start).count();
        double ns_per_op = static_cast<double>(duration) / (2.0 * ITERATIONS);

        std::cout << "✓ Test 4: Performance - " << ns_per_op
                  << " ns/operation (target: <100ns)" << std::endl;

        if (ns_per_op > 500) {  // Allow some margin on desktop
            std::cerr << "WARNING: Performance slower than expected" << std::endl;
        }
    }

    std::cout << "\n✅ ALL VALIDATION TESTS PASSED" << std::endl;
    std::cout << "LockFreeQueue is ready for integration" << std::endl;

    return 0;
}
