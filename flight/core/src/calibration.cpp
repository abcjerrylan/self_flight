#include "self_flight/core/calibration.hpp"
#include <algorithm>
#include <cmath>

namespace self_flight::core {
namespace {
Vec3 product(Vec3 a, Vec3 b) { return {a.x*b.x, a.y*b.y, a.z*b.z}; }
float norm(Vec3 v) { return std::sqrt(dot(v,v)); }
float maximum(Vec3 v) { return std::max({v.x,v.y,v.z}); }
float axis(Vec3 v, unsigned index) { return index == 0 ? v.x : index == 1 ? v.y : v.z; }
bool between(float value, float lower, float upper) { return value >= lower && value <= upper; }
}
void VectorMoments::add(Vec3 value) {
    ++count;
    const auto delta = value - mean;
    mean = mean + delta * (1.0F / static_cast<float>(count));
    m2 = m2 + product(delta, value - mean);
}
Vec3 VectorMoments::variance() const { return count > 1 ? m2 * (1.0F / (count-1)) : Vec3{}; }
void StationaryCalibration::reset() { *this = StationaryCalibration{}; }

CalibrationError StationaryCalibration::add(bool gyro, const VectorSample& sample) {
    const unsigned index = gyro ? 1 : 0;
    auto reject = [&](CalibrationError error) { reset(); return error; };
    const auto& meta = sample.metadata;
    if (!is_usable(sample, meta.available_us, 5000)) return reject(CalibrationError::InvalidSample);
    if (moments_[index].count && (meta.sequence - last_[index].sequence != 1 ||
        !checked_delta(last_[index].measured_us, meta.measured_us, gyro ? 3000 : 4000).valid()))
        return reject(CalibrationError::Timing);
    const unsigned other = 1-index;
    if (moments_[other].count && !is_fresh(last_[other].measured_us, meta.available_us, 5000))
        return reject(CalibrationError::Timing);
    if ((gyro && norm(sample.value) > 0.20F) || (!gyro && !between(norm(sample.value), 8.5F, 11.0F)))
        return reject(CalibrationError::Motion);
    if (!moments_[index].count) first_[index] = meta.measured_us;
    moments_[index].add(sample.value);
    last_[index] = meta;
    const auto& stats = moments_[index];
    if (stats.count >= 30 && ((gyro && (norm(stats.mean) > 0.05F || maximum(stats.variance()) > 0.0009F)) ||
        (!gyro && (maximum(stats.variance()) > 0.04F || !between(norm(stats.mean), 8.8F, 10.8F)))))
        return reject(CalibrationError::Motion);
    return CalibrationError::None;
}

bool StationaryCalibration::estimate(GyroEstimate& out) const {
    if (moments_[1].count < 2500 || moments_[0].count < 2000) return false;
    const auto begin = std::max(first_[0],first_[1]);
    const auto end = std::min(last_[0].measured_us,last_[1].measured_us);
    if (end < begin || end-begin < 3000000) return false;
    out = {moments_[1].mean, moments_[1].variance(), moments_[0].mean, end};
    return true;
}

bool calibration_parameters_valid(const Calibration& c) {
    return c.format_version == 1 && is_finite(c.gyro_bias_rad_s) && norm(c.gyro_bias_rad_s) <= 0.05F &&
        is_finite(c.accel_bias_m_s2) && norm(c.accel_bias_m_s2) <= 1.0F && is_finite(c.accel_scale) &&
        between(c.accel_scale.x,0.9F,1.1F) && between(c.accel_scale.y,0.9F,1.1F) &&
        between(c.accel_scale.z,0.9F,1.1F) && std::isfinite(c.stationary_gyro_variance) &&
        between(c.stationary_gyro_variance,0.0F,0.0009F) &&
        (!c.temperature_valid || (std::isfinite(c.temperature_c) && between(c.temperature_c,-40,85)));
}

CalibrationError fit_accelerometer(const std::array<Vec3,6>& means, Calibration& out) {
    float bias[3]{}, scale[3]{};
    for (unsigned i=0; i<6; ++i) {
        if (!is_finite(means[i])) return CalibrationError::InvalidSample;
        for (unsigned j=0; j<3; ++j) {
            const float value = axis(means[i],j);
            if (j != i/2 ? std::fabs(value) > 0.8F :
                !between(value * (i%2 ? -1 : 1), 8.8F,10.8F)) return CalibrationError::Face;
        }
    }
    for (unsigned i=0; i<3; ++i) {
        const float plus = axis(means[2*i],i), minus = axis(means[2*i+1],i);
        bias[i] = (plus+minus)*0.5F;
        scale[i] = 2*kGravity/(plus-minus);
    }
    auto candidate = out;
    candidate.accel_bias_m_s2 = {bias[0],bias[1],bias[2]};
    candidate.accel_scale = {scale[0],scale[1],scale[2]};
    if (!calibration_parameters_valid(candidate)) return CalibrationError::Parameters;
    for (const auto mean : means) {
        const float corrected = norm(product(mean-candidate.accel_bias_m_s2,candidate.accel_scale));
        if (std::fabs(corrected-kGravity) > 0.15F) return CalibrationError::Face;
    }
    out = candidate;
    return CalibrationError::None;
}

bool apply_calibration(Vec3 input, bool gyro, const Calibration& c, Vec3& out) {
    if (c.quality != CalibrationQuality::Accepted || !calibration_parameters_valid(c) || !is_finite(input)) return false;
    const auto value = gyro ? input-c.gyro_bias_rad_s : product(input-c.accel_bias_m_s2,c.accel_scale);
    if (!is_finite(value)) return false;
    out = value;
    return true;
}
}
