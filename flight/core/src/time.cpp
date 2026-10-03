#include "self_flight/core/time.hpp"

namespace self_flight::core {

TimeDelta checked_delta(TimestampUs previous, TimestampUs current,
                        TimestampUs maximum_gap_us) noexcept {
    if (maximum_gap_us == 0U) { return {TimeError::InvalidLimit, 0.0F}; }
    if (current == previous) { return {TimeError::Duplicate, 0.0F}; }
    if (current < previous) { return {TimeError::Backward, 0.0F}; }
    const TimestampUs elapsed = current - previous;
    if (elapsed > maximum_gap_us) { return {TimeError::GapExceeded, 0.0F}; }
    return {TimeError::None, static_cast<float>(elapsed) * 1.0e-6F};
}

bool is_fresh(TimestampUs measured, TimestampUs now,
               TimestampUs maximum_age_us) noexcept {
    return measured <= now && (now - measured) <= maximum_age_us;
}

} // namespace self_flight::core
