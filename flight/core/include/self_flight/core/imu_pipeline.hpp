#pragma once
#include "self_flight/core/attitude.hpp"

namespace self_flight::core {
// One gyro low-pass shared by attitude propagation and future rate control.
class ImuPipeline {
public:
    explicit ImuPipeline(AttitudeConfig attitude = {}, float derivative_cutoff_hz = 30);
    AttitudeError update(const ImuSample&, TimestampUs now);
    const AttitudeState& state() const { return state_; }
    const RateFeedback& rate() const { return rate_; }
    RateError rate_error() const { return rate_error_; }
    float accel_weight() const { return estimator_.accel_weight(); }
    float dt_s() const { return estimator_.dt_s(); }
private:
    Mahony estimator_;
    RateFeedbackFilter filter_;
    AttitudeState state_{};
    RateFeedback rate_{};
    RateError rate_error_{RateError::None};
};
}
