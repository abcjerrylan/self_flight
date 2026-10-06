#pragma once

// Design declarations only; not built into firmware. No controller implementation.
// Body FRD, navigation NED, SI; q_nb maps body to navigation.
#include "self_flight/core/data.hpp"

namespace self_flight::proposal {
using namespace core;

struct RateFeedback {
    SampleMetadata metadata{};
    Vec3 filtered_gyro_rad_s{}; // Calibrated gyro before online residual bias.
    Vec3 body_rate_rad_s{};    // Filtered gyro minus residual bias, no gravity correction.
    Vec3 angular_accel_rad_s2{}; // Differentiate filtered gyro, then D low-pass.
    bool derivative_valid{};
};

struct AttitudeTarget {
    SampleMetadata metadata{};
    Quaternion q_nb{};
    Vec3 feedforward_body_rate_rad_s{};
    float collective{};
    std::uint32_t reference_epoch{};
};

struct RateTarget {
    SampleMetadata metadata{};
    Vec3 body_rate_rad_s{};
    float collective{};
};

struct ControlDemand {
    SampleMetadata metadata{};
    Vec3 effort{}; // Normalized roll/pitch/yaw control, not identified N.m.
    float collective{};
};

struct AllocationFeedback {
    SampleMetadata metadata{};
    Vec3 unallocated_effort{}; // Requested minus achieved.
    std::array<bool, 3> positive_limited{};
    std::array<bool, 3> negative_limited{};
};

struct AllocationResult {
    ActuatorCommand motors{}; // Allocator leaves output_allowed false; guard owns permission.
    AllocationFeedback feedback{};
};

struct RateFilterConfig {
    float gyro_cutoff_hz{};
    float derivative_cutoff_hz{};
    TimestampUs maximum_gap_us{}; // Zero means unconfigured, never a flight default.
};
struct RateFilterState {
    SampleMetadata previous{};
    Vec3 gyro{}, derivative{};
    bool ready{};
};

// Caller supplies time. Failure leaves out unchanged; gaps require explicit reset.
// Feed filtered_gyro to Mahony with its internal gyro filter bypassed.
bool update_rate_feedback(const RateFilterConfig&, RateFilterState&,
                          const VectorSample& calibrated_body_gyro,
                          Vec3 residual_bias, TimestampUs now, RateFeedback& out);
void reset(RateFilterState&);

struct AttitudeControlConfig {
    Vec3 kp{}, maximum_rate_rad_s{};
};
// Full-quaternion P prototype, not PX4's reduced-attitude/yaw-weight algorithm.
bool attitude_control(const AttitudeControlConfig&, const AttitudeState&,
                      const AttitudeTarget&, TimestampUs now, RateTarget& out);

struct RateControlConfig {
    Vec3 kp{}, ki{}, kd{}, integral_limit{};
    TimestampUs maximum_gap_us{}, maximum_feedback_age_us{};
};
struct RateControlState {
    SampleMetadata previous{};
    Vec3 integral{};
};
// dt is checked against actual measurement times; missing time is rejected, not clamped.
// integrate=false freezes I; disarm, fault and reference reset explicitly clear it.
// Stale allocation feedback inhibits I. Unready D feedback yields no valid control.
bool rate_control(const RateControlConfig&, RateControlState&, const RateFeedback&,
                  const RateTarget&, const AllocationFeedback& previous_allocation,
                  TimestampUs now, float dt_s, bool integrate, ControlDemand& out);
void reset(RateControlState&);

using MixingMatrix = std::array<std::array<float, 4>, 4>;
class QuadMixer {
public:
    // Electrical M1..M4 rows, collective/roll/pitch/yaw columns.
    // Derive signs from actual arm positions and spin; cache inverse at configure time.
    bool configure(const MixingMatrix& motor_from_control);
    bool allocate(const ControlDemand&, TimestampUs now, AllocationResult& out) const;
private:
    MixingMatrix forward_{}, inverse_{};
    bool configured_{};
};

struct GuardInput {
    TimestampUs now{}, maximum_rc_age_us{};
    std::uint32_t attitude_epoch{};
    bool calibration_accepted{}, imu_healthy{}, attitude_healthy{}, control_healthy{};
};
struct GuardState {
    FlightStatus status{};
    std::uint32_t reference_epoch{};
    bool previous_arm_request{};
};
// Explicit arm edge + low throttle + health. Fault latches; never auto-rearms.
// Epoch change while armed faults. Driver also checks permission and command freshness.
void update_guard(GuardState&, const PilotCommand&, const GuardInput&);

} // namespace self_flight::proposal
