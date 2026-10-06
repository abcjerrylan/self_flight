#pragma once
#include "self_flight/core/data.hpp"

namespace self_flight::sbus {
struct Frame {
    core::SampleMetadata metadata{}; // Complete RC frame reception time, not first byte.
    std::array<std::uint16_t,16> channels{};
    bool digital17{}, digital18{}, frame_lost{}, failsafe{};
};
struct Statistics {
    std::uint32_t frames{}, malformed{}, partial_resets{}, time_errors{}, lost{}, failsafe{};
};
class Parser {
public:
    // UART supplies de-inverted 8-bit data; parity/framing errors must reset the parser.
    // Equal timestamps allow DMA chunks; backward time rejects bytes. Failure preserves out.
    bool push(std::uint8_t byte, core::TimestampUs received_us, Frame& out);
    void reset(); // Flush partial frame, preserving cumulative statistics/sequence.
    const Statistics& stats() const { return stats_; }
private:
    std::array<std::uint8_t,25> bytes_{};
    unsigned used_{};
    core::TimestampUs last_time_{};
    bool seen_{};
    Statistics stats_{};
};

constexpr std::uint8_t kUnassigned=255;
struct ChannelMap {
    // Explicit zero-based channel indices; defaults intentionally unassigned.
    std::uint8_t roll{kUnassigned}, pitch{kUnassigned}, yaw{kUnassigned}, throttle{kUnassigned}, arm{kUnassigned};
    std::uint8_t attitude_mode{kUnassigned}, emergency_stop{kUnassigned};
    std::uint16_t low{}, center{}, high{}; // Measure transmitter endpoints before configuring.
    std::array<bool,3> reversed{};
};
bool valid_mapping(const ChannelMap&);
// Frame-lost alone is not failsafe: do not refresh the previous good command.
// Failsafe frames do publish receiver_failsafe=true. Failure leaves out unchanged.
bool map_pilot(const Frame&, const ChannelMap&, core::PilotCommand& out);
}
