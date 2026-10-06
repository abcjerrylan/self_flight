#include "self_flight/core/flight_guard.hpp"
#include <cmath>

namespace self_flight::core {
namespace {
bool metadata_usable(const SampleMetadata& m, TimestampUs now, TimestampUs age) {
    return m.valid && m.measured_us <= m.available_us && m.available_us <= now && is_fresh(m.measured_us,now,age);
}
}
bool is_usable(const PilotCommand& p, TimestampUs now, TimestampUs age) {
    return metadata_usable(p.metadata,now,age) && !p.receiver_failsafe &&
        std::isfinite(p.roll) && std::fabs(p.roll) <= 1 &&
        std::isfinite(p.pitch) && std::fabs(p.pitch) <= 1 &&
        std::isfinite(p.yaw) && std::fabs(p.yaw) <= 1 &&
        std::isfinite(p.collective) && p.collective >= 0 && p.collective <= 1 &&
        (p.mode == FlightMode::Rate || p.mode == FlightMode::Attitude);
}
FlightGuard::FlightGuard(GuardConfig c) : config_(c) {
    configured_=c.maximum_rc_age_us > 0 && c.maximum_sensor_age_us > 0 && c.maximum_update_gap_us > 0 &&
        std::isfinite(c.arming_throttle_limit) && c.arming_throttle_limit >= 0 && c.arming_throttle_limit <= 0.1F;
}
const FlightStatus& FlightGuard::update(const PilotCommand& p, const FlightHealth& h, TimestampUs now) {
    std::uint32_t reasons=0;
    const bool clock_ok=!seen_ || checked_delta(previous_time_,now,config_.maximum_update_gap_us).valid();
    if (!configured_) reasons |= InvalidGuardConfig;
    if (!clock_ok) reasons |= GuardTiming;
    if (!h.calibration_accepted) reasons |= CalibrationPending;
    const bool rc_ok=is_usable(p,now,config_.maximum_rc_age_us);
    if (!rc_ok) reasons |= ReceiverInvalid;
    Quaternion unit;
    if (!metadata_usable(h.attitude.metadata,now,config_.maximum_sensor_age_us) ||
        !try_normalize(h.attitude.q_nb,unit) || !is_finite(h.attitude.body_rate_rad_s)) reasons |= AttitudeInvalid;
    if (!metadata_usable(h.rate.metadata,now,config_.maximum_sensor_age_us) ||
        !h.rate.derivative_valid || !is_finite(h.rate.body_rate_rad_s) ||
        !is_finite(h.rate.angular_accel_rad_s2) ||
        h.rate.metadata.sequence != h.attitude.metadata.sequence ||
        h.rate.metadata.measured_us != h.attitude.metadata.measured_us) reasons |= RateInvalid;
    if (!h.controller_ready) reasons |= ControllerUnavailable;
    if (!h.output_driver_ready) reasons |= OutputUnavailable;
    if (rc_ok && p.collective > config_.arming_throttle_limit) reasons |= ThrottleHigh;
    // A fresh decoded emergency stop remains actionable even on a failsafe frame.
    if (metadata_usable(p.metadata,now,config_.maximum_rc_age_us) && p.emergency_stop) reasons |= EmergencyStop;
    if (status_.state == FlightState::Armed && h.attitude.total_epoch != reference_epoch_) reasons |= ReferenceChanged;
    const bool arm_edge=rc_ok && p.arm_request && !previous_arm_ && arm_off_seen_;
    const auto faults=reasons & ~static_cast<std::uint32_t>(ThrottleHigh);
    const auto previous_state=status_.state;
    switch (status_.state) {
    case FlightState::Boot:
        status_.state=FlightState::Calibrating;
        break;
    case FlightState::Calibrating:
        if (h.calibration_accepted) status_.state=FlightState::Disarmed;
        break;
    case FlightState::Disarmed:
        if (arm_edge && reasons == 0) {
            status_.state=FlightState::Armed;
            reference_epoch_=h.attitude.total_epoch;
        }
        break;
    case FlightState::Armed:
        if (faults) { status_.state=FlightState::Fault; status_.fault_reasons |= faults; }
        else if (rc_ok && !p.arm_request) status_.state=FlightState::Disarmed;
        break;
    case FlightState::Fault:
        if (reasons == 0 && rc_ok && !p.arm_request) {
            status_.state=FlightState::Disarmed;
            status_.fault_reasons=0;
        }
        break;
    }
    if (status_.state != previous_state && (previous_state == FlightState::Armed || status_.state == FlightState::Armed))
        ++reset_sequence_;
    // Seeing arm HIGH during boot, calibration or a rejected attempt consumes that edge.
    if (rc_ok) { previous_arm_=p.arm_request; arm_off_seen_ |= !p.arm_request; }
    // A forward gap faults this cycle but establishes a baseline for explicit recovery.
    if (!seen_ || now > previous_time_) { previous_time_=now; seen_=true; }
    status_.arm_block_reasons=reasons;
    status_.output_allowed=status_.state == FlightState::Armed && faults == 0;
    status_.metadata={now,now,status_.metadata.sequence+1,configured_ && clock_ok};
    return status_;
}
void FlightGuard::deadline_missed(TimestampUs now) {
    status_.output_allowed=false;
    status_.arm_block_reasons |= ControlDeadline;
    if (status_.state == FlightState::Armed) {
        status_.state=FlightState::Fault; status_.fault_reasons |= ControlDeadline;
        ++reset_sequence_;
    }
    if (!seen_ || now > previous_time_) { previous_time_=now; seen_=true; }
    status_.metadata={now,now,status_.metadata.sequence+1,false};
}
}
