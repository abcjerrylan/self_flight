#pragma once

#include <cstdint>

namespace self_flight::core {

using TimestampUs = std::uint64_t;

enum class TimeError : std::uint8_t {
    None, Duplicate, Backward, GapExceeded, InvalidLimit
};

struct TimeDelta {
    TimeError error{TimeError::InvalidLimit};
    float seconds{0.0F};
    bool valid() const noexcept { return error == TimeError::None; }
};

TimeDelta checked_delta(TimestampUs previous, TimestampUs current,
                        TimestampUs maximum_gap_us) noexcept;
// Timestamp zero is a valid epoch. Caller must also check sample validity.
bool is_fresh(TimestampUs measured, TimestampUs now,
               TimestampUs maximum_age_us) noexcept;

} // namespace self_flight::core
