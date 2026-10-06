#include "self_flight/core/imu_pipeline.hpp"

namespace self_flight::core {
namespace {
AttitudeConfig bypass_gyro(AttitudeConfig c) { c.gyro_cutoff_hz=0; return c; }
}
ImuPipeline::ImuPipeline(AttitudeConfig attitude, float derivative_cutoff)
    : estimator_(bypass_gyro(attitude)),
      filter_({attitude.gyro_cutoff_hz,derivative_cutoff,attitude.maximum_gyro_gap_us,5000}) {}
AttitudeError ImuPipeline::update(const ImuSample& sample, TimestampUs now) {
    state_.metadata.valid=rate_.metadata.valid=rate_.derivative_valid=false;
    RateFeedback feedback;
    rate_error_=filter_.update(sample.gyro,estimator_.state().gyro_bias_rad_s,now,feedback);
    if (rate_error_ != RateError::None) {
        // Let Mahony see the same discontinuity and reset its reference as before.
        if (rate_error_ == RateError::Timing) estimator_.update(sample,now);
        return rate_error_ == RateError::Timing ? AttitudeError::Timing :
               rate_error_ == RateError::Config ? AttitudeError::Config :
               rate_error_ == RateError::Numerical ? AttitudeError::Numerical : AttitudeError::InvalidSample;
    }
    auto filtered=sample;
    filtered.gyro.value=feedback.filtered_gyro_rad_s;
    const auto error=estimator_.update(filtered,now);
    state_=estimator_.state();
    // Match the same frame's updated residual bias, without differentiating that bias.
    feedback.body_rate_rad_s=feedback.filtered_gyro_rad_s-state_.gyro_bias_rad_s;
    feedback.metadata.valid &= state_.metadata.valid && is_finite(feedback.body_rate_rad_s);
    feedback.derivative_valid &= feedback.metadata.valid;
    rate_=feedback;
    return error;
}
}
