#include "self_flight/core/rate_feedback.hpp"
#include <cmath>

namespace self_flight::core {
bool LowPass3::update(Vec3 input, float dt, float cutoff, Vec3& out) {
    if (!is_finite(input) || !std::isfinite(dt) || dt < 0 || (ready_ && dt == 0) ||
        !std::isfinite(cutoff) || cutoff < 0) return false;
    const float alpha=cutoff == 0 ? 1.0F : dt/(dt+1.0F/(2*kPi*cutoff));
    const auto value=ready_ ? value_+(input-value_)*alpha : input;
    if (!is_finite(value)) return false;
    value_=value; ready_=true; out=value;
    return true;
}
RateFeedbackFilter::RateFeedbackFilter(RateFeedbackConfig c) : config_(c) {
    configured_=std::isfinite(c.gyro_cutoff_hz) && c.gyro_cutoff_hz >= 0 && c.gyro_cutoff_hz <= 500 &&
        std::isfinite(c.derivative_cutoff_hz) && c.derivative_cutoff_hz >= 0 && c.derivative_cutoff_hz <= 500 &&
        c.maximum_gap_us > 0 && c.maximum_age_us > 0;
}
void RateFeedbackFilter::reset() {
    previous_={}; gyro_filter_.reset(); derivative_filter_.reset(); gyro_={}; ready_=false;
}
RateError RateFeedbackFilter::update(const VectorSample& sample, Vec3 bias,
                                     TimestampUs now, RateFeedback& out) {
    if (!configured_) return RateError::Config;
    if (!is_usable(sample,now,config_.maximum_age_us) || !is_finite(bias)) return RateError::InvalidSample;
    float dt=0;
    if (ready_) {
        const auto delta=checked_delta(previous_.measured_us,sample.metadata.measured_us,config_.maximum_gap_us);
        const auto step=sample.metadata.sequence-previous_.sequence;
        if (!delta.valid() || step != 1) {
            if (sample.metadata.measured_us > previous_.measured_us && step != 0 && step < 0x80000000U) reset();
            return RateError::Timing;
        }
        dt=delta.seconds;
    }
    // Commit filters only after the complete frame is finite.
    auto gyro_filter=gyro_filter_, derivative_filter=derivative_filter_;
    Vec3 gyro, derivative;
    if (!gyro_filter.update(sample.value,dt,config_.gyro_cutoff_hz,gyro)) return RateError::Numerical;
    if (!ready_ && !derivative_filter.update({},0,config_.derivative_cutoff_hz,derivative)) return RateError::Numerical;
    if (ready_ && !derivative_filter.update((gyro-gyro_)*(1/dt),dt,
                                           config_.derivative_cutoff_hz,derivative)) return RateError::Numerical;
    const auto rate=gyro-bias;
    if (!is_finite(rate)) return RateError::Numerical;
    out={sample.metadata,gyro,rate,derivative,ready_};
    gyro_filter_=gyro_filter; derivative_filter_=derivative_filter;
    previous_=sample.metadata; gyro_=gyro; ready_=true;
    return RateError::None;
}
}
