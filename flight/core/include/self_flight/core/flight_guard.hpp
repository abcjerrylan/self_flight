#pragma once
#include "self_flight/core/data.hpp"
#include "self_flight/core/rate_feedback.hpp"

namespace self_flight::core {
enum GuardReason : std::uint32_t {
    CalibrationPending=1U<<0, ReceiverInvalid=1U<<1, AttitudeInvalid=1U<<2,
    RateInvalid=1U<<3, ControllerUnavailable=1U<<4, OutputUnavailable=1U<<5,
    ThrottleHigh=1U<<6, EmergencyStop=1U<<7, ReferenceChanged=1U<<8,
    GuardTiming=1U<<9, InvalidGuardConfig=1U<<10, ControlDeadline=1U<<11
};
struct GuardConfig {
    TimestampUs maximum_rc_age_us{100000}, maximum_sensor_age_us{5000}, maximum_update_gap_us{5000};
    float arming_throttle_limit{0.05F};
};
struct FlightHealth {
    AttitudeState attitude{};
    RateFeedback rate{};
    bool calibration_accepted{}, controller_ready{}, output_driver_ready{};
};
bool is_usable(const PilotCommand&, TimestampUs now, TimestampUs maximum_age_us);
class FlightGuard {
public:
    explicit FlightGuard(GuardConfig config = {});
    // Fault recovery requires healthy inputs, low throttle and a fresh arm-OFF command.
    const FlightStatus& update(const PilotCommand&, const FlightHealth&, TimestampUs now);
    void deadline_missed(TimestampUs now);
    const FlightStatus& status() const { return status_; }
    std::uint32_t reset_sequence() const { return reset_sequence_; }
private:
    GuardConfig config_;
    FlightStatus status_{};
    TimestampUs previous_time_{};
    std::uint32_t reference_epoch_{};
    std::uint32_t reset_sequence_{}; // Future controllers clear I/targets when this changes.
    bool configured_{}, seen_{}, previous_arm_{}, arm_off_seen_{};
};
}
