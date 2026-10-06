#pragma once
#include "self_flight/core/data.hpp"

namespace self_flight::core {
class LowPass3 {
public:
    bool update(Vec3 input, float dt_s, float cutoff_hz, Vec3& out);
    void reset() { *this = {}; }
private:
    Vec3 value_{};
    bool ready_{};
};

struct RateFeedback {
    SampleMetadata metadata{};
    Vec3 filtered_gyro_rad_s{}; // Before online residual bias, for Mahony and differentiation.
    Vec3 body_rate_rad_s{};
    Vec3 angular_accel_rad_s2{};
    bool derivative_valid{};
};
struct RateFeedbackConfig {
    float gyro_cutoff_hz{80}, derivative_cutoff_hz{30};
    TimestampUs maximum_gap_us{3000}, maximum_age_us{5000};
};
enum class RateError : std::uint8_t { None, Config, InvalidSample, Timing, Numerical };
class RateFeedbackFilter {
public:
    explicit RateFeedbackFilter(RateFeedbackConfig config = {});
    // Rejected inputs leave out unchanged. Positive gaps reset history; never integrate missing time.
    RateError update(const VectorSample& calibrated_body_gyro, Vec3 residual_bias,
                     TimestampUs now, RateFeedback& out);
    void reset();
private:
    RateFeedbackConfig config_;
    SampleMetadata previous_{};
    LowPass3 gyro_filter_, derivative_filter_;
    Vec3 gyro_{};
    bool configured_{}, ready_{};
};
}
