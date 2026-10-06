#pragma once
#include <cstdint>
#include "bmi088.hpp"

namespace platform {
void start_timer();
bool start_receiver();
std::uint32_t cpu_hz();
// Serialized across ISR/threads; read at least once per TIM2 wrap (71.6 minutes).
std::uint64_t time_us();
bool write(const char* text);
self_flight::bmi088::Bus imu_bus();
struct CriticalSection {
    CriticalSection();
    ~CriticalSection();
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
private:
    std::uint32_t interrupts;
};
}
