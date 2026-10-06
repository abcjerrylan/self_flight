#pragma once
#include <cstdint>

namespace self_flight::rc {
struct Statistics { std::uint32_t bytes{}, errors{}, drops{}, restarts{}; };
unsigned start();
void received(std::uint8_t byte, std::uint32_t error=0); // UART ISR: enqueue only.
Statistics statistics();
}
