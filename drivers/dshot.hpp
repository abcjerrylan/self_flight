#pragma once
#include "self_flight/core/data.hpp"

namespace self_flight::dshot {
// Unidirectional DShot300; 1..47 ESC commands are intentionally not accepted.
bool encode(std::uint16_t value, bool request_telemetry, std::uint16_t& out);
enum StopReason : std::uint32_t { Permission=1U<<0, Stale=1U<<1, Invalid=1U<<2 };
struct Batch {
    core::SampleMetadata metadata{};
    std::array<std::uint16_t,4> frames{}; // Electrical M1..M4, prepared as one batch.
    std::uint32_t stop_reasons{Permission};
};
// Zero normalized throttle maps to STOP; >0 maps to 48..2047. No armed idle yet.
// Rejected permission/data returns a complete four-stop batch, never partial output.
Batch prepare(const core::ActuatorCommand&, const core::FlightStatus&,
              core::TimestampUs now, core::TimestampUs maximum_age_us = 3000);
}
