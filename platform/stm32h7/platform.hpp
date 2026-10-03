#pragma once
#include <cstdint>

namespace platform {
void start_timer();
std::uint32_t cpu_hz();
// Call from the single startup thread at least once per TIM2 wrap (71.6 minutes).
std::uint64_t time_us();
void write(const char* text);
}
