#pragma once
#include "self_flight/core/data.hpp"

namespace self_flight::core {
// First-order RC low-pass, coefficients derived from this sample's actual dt.
class LowPass3 {
public:
    bool update(Vec3 input, float dt_s, float cutoff_hz, Vec3& out);
    void reset() { *this = {}; }
private:
    Vec3 value_{};
    bool ready_{};
};
bool propagate(Quaternion q_nb, Vec3 body_rate_rad_s, float dt_s, Quaternion& out);
bool attitude_euler(Quaternion q_nb, Vec3& roll_pitch_yaw_rad);

struct AttitudeConfig {
    float gyro_cutoff_hz{80}, accel_cutoff_hz{20}; // 0 bypasses the software filter.
    float kp{2}, ki{0.1F}, maximum_bias_rad_s{0.05F};
    float accel_norm_tolerance{0.15F}; // Fraction of g; weight is zero beyond it.
    TimestampUs maximum_gyro_gap_us{3000}, maximum_accel_gap_us{4000};
};
enum class AttitudeError : std::uint8_t { None, WaitingForAccel, InvalidSample, Timing, Config, Numerical };

// Six-axis Mahony. Startup bias has already been removed; bias here is residual, in body axes.
class Mahony {
public:
    explicit Mahony(AttitudeConfig config = {});
    AttitudeError update(const ImuSample& sample, TimestampUs now);
    const AttitudeState& state() const { return state_; }
    float accel_weight() const { return weight_; }
    float dt_s() const { return dt_; }
private:
    AttitudeConfig config_;
    AttitudeState state_{};
    LowPass3 gyro_filter_, accel_filter_;
    Vec3 accel_{};
    TimestampUs accel_time_{};
    std::uint32_t accel_sequence_{};
    float weight_{}, dt_{}, yaw_rad_{};
    bool config_valid_{}, initialized_{}, accel_seen_{};
};
}
