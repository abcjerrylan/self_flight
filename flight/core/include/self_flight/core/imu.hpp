#pragma once
#include "self_flight/core/data.hpp"

namespace self_flight::core {
// One cursor per consumer. Sample dt/gap checks belong to the attitude solver.
class ImuCursor {
public:
    bool take(const ImuSample& latest, TimestampUs now, ImuSample& out) {
        const auto& g = latest.gyro.metadata;
        if (!is_usable(latest.gyro,now,5000) || (gyro_seen_ &&
            (!forward(gyro_sequence_,g.sequence) || g.measured_us <= gyro_time_))) return false;
        out = latest;
        const auto& a = latest.accel.metadata;
        out.accel.metadata.valid = is_usable(latest.accel,now,5000) && a.measured_us <= g.measured_us;
        out.accel_is_new = out.accel.metadata.valid && (!accel_seen_ ||
            (forward(accel_sequence_,a.sequence) && a.measured_us > accel_time_));
        if (out.accel_is_new) {
            accel_sequence_ = a.sequence; accel_time_ = a.measured_us; accel_seen_ = true;
        }
        gyro_sequence_ = g.sequence; gyro_time_ = g.measured_us; gyro_seen_ = true;
        return true;
    }
private:
    static bool forward(std::uint32_t before, std::uint32_t after) {
        const auto step = after-before;
        return step != 0 && step < 0x80000000U;
    }
    std::uint32_t gyro_sequence_{}, accel_sequence_{};
    TimestampUs gyro_time_{}, accel_time_{};
    bool gyro_seen_{}, accel_seen_{};
};
}
