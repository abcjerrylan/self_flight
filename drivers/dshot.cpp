#include "dshot.hpp"
#include <cmath>

namespace self_flight::dshot {
bool encode(std::uint16_t value, bool telemetry, std::uint16_t& out) {
    if (value > 2047 || (value != 0 && value < 48)) return false;
    const auto payload=static_cast<std::uint16_t>((value<<1) | static_cast<unsigned>(telemetry));
    const auto checksum=static_cast<std::uint16_t>((payload ^ (payload>>4) ^ (payload>>8)) & 15);
    out=static_cast<std::uint16_t>((payload<<4) | checksum);
    return true;
}
Batch prepare(const core::ActuatorCommand& cmd, const core::FlightStatus& status,
              core::TimestampUs now, core::TimestampUs age) {
    Batch batch{{now,now,cmd.metadata.sequence,true},{},0};
    if (!cmd.output_allowed || !status.output_allowed || status.state != core::FlightState::Armed)
        batch.stop_reasons |= Permission;
    for (const auto& m : {cmd.metadata,status.metadata}) {
        if (!m.valid || m.measured_us > m.available_us) batch.stop_reasons |= Invalid;
        if (m.available_us > now || age == 0 || !core::is_fresh(m.measured_us,now,age)) batch.stop_reasons |= Stale;
    }
    for (float v : cmd.motor_normalized)
        if (!std::isfinite(v) || v < 0 || v > 1) batch.stop_reasons |= Invalid;
    if (batch.stop_reasons) return batch;
    for (unsigned i=0;i<4;++i) {
        const float v=cmd.motor_normalized[i];
        const auto value=static_cast<std::uint16_t>(v == 0 ? 0 : 48+std::lround(v*1999));
        encode(value,false,batch.frames[i]);
    }
    return batch;
}
}
