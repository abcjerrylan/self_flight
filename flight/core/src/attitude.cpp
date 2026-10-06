#include "self_flight/core/attitude.hpp"
#include "self_flight/core/calibration.hpp"
#include <algorithm>
#include <cmath>

namespace self_flight::core {
namespace {
float norm(Vec3 value) { return std::sqrt(dot(value,value)); }
float yaw_of_unit(Quaternion q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
Vec3 euler_of_unit(Quaternion q) {
    return {std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
            std::asin(std::clamp(2*(q.w*q.y-q.z*q.x),-1.0F,1.0F)),
            yaw_of_unit(q)};
}
Vec3 down_in_body(Quaternion q) {
    return {2*(q.x*q.z-q.w*q.y),2*(q.y*q.z+q.w*q.x),1-2*(q.x*q.x+q.y*q.y)};
}
Quaternion tilt(Vec3 down) {
    const float r=std::atan2(down.y,down.z)*0.5F;
    const float p=std::atan2(-down.x,std::hypot(down.y,down.z))*0.5F;
    return {std::cos(r)*std::cos(p),std::sin(r)*std::cos(p),
            std::cos(r)*std::sin(p),-std::sin(r)*std::sin(p)};
}
}
bool propagate(Quaternion q, Vec3 rate, float dt, Quaternion& out) {
    Quaternion unit;
    if (!is_finite(rate) || !std::isfinite(dt) || dt <= 0 || !try_normalize(q,unit)) return false;
    const auto step=multiply(unit,{1,rate.x*dt*0.5F,rate.y*dt*0.5F,rate.z*dt*0.5F});
    return try_normalize(step,out);
}
bool attitude_euler(Quaternion q, Vec3& out) {
    Quaternion unit;
    if (!try_normalize(q,unit)) return false;
    out=euler_of_unit(unit);
    return true;
}
Mahony::Mahony(AttitudeConfig c) : config_(c) {
    config_valid_=std::isfinite(c.gyro_cutoff_hz) && c.gyro_cutoff_hz >= 0 && c.gyro_cutoff_hz <= 500 &&
        std::isfinite(c.accel_cutoff_hz) && c.accel_cutoff_hz >= 0 && c.accel_cutoff_hz <= 400 &&
        std::isfinite(c.kp) && c.kp >= 0 && std::isfinite(c.ki) && c.ki >= 0 &&
        std::isfinite(c.maximum_bias_rad_s) && c.maximum_bias_rad_s > 0 &&
        std::isfinite(c.accel_norm_tolerance) && c.accel_norm_tolerance > 0 && c.accel_norm_tolerance < 1 &&
        c.maximum_gyro_gap_us > 0 && c.maximum_accel_gap_us > 0;
}
AttitudeError Mahony::update(const ImuSample& sample, TimestampUs now) {
    state_.metadata.valid=false;
    weight_=dt_=0;
    if (!config_valid_) return AttitudeError::Config;
    if (!is_usable(sample.gyro,now,5000)) return AttitudeError::InvalidSample;
    const auto& g=sample.gyro.metadata;
    if (initialized_) {
        const auto delta=checked_delta(state_.gyro_measured_us,g.measured_us,config_.maximum_gyro_gap_us);
        const auto step=g.sequence-state_.metadata.sequence;
        if (!delta.valid() || step != 1) {
            // Never propagate across missing time. Re-align tilt on the next trusted frame.
            if (g.measured_us > state_.gyro_measured_us && step != 0 && step < 0x80000000U) {
                initialized_=accel_seen_=false;
                gyro_filter_.reset(); accel_filter_.reset();
            }
            return AttitudeError::Timing;
        }
        dt_=delta.seconds;
    }
    const auto& a=sample.accel.metadata;
    const bool usable=is_usable(sample.accel,now,5000) && a.measured_us <= g.measured_us;
    if (usable && (!accel_seen_ || (sample.accel_is_new && a.sequence != accel_sequence_))) {
        const auto delta=checked_delta(accel_time_,a.measured_us,config_.maximum_accel_gap_us);
        if (!accel_seen_ || !delta.valid() || a.sequence-accel_sequence_ != 1) accel_filter_.reset();
        const float magnitude=norm(sample.accel.value);
        const float trust=std::clamp(1-std::fabs(magnitude/kGravity-1)/config_.accel_norm_tolerance,0.0F,1.0F);
        if (trust > 0) {
            if (!accel_filter_.update(sample.accel.value,delta.valid() ? delta.seconds : 0,
                                     config_.accel_cutoff_hz,accel_)) return AttitudeError::Numerical;
        } else accel_filter_.reset(); // A rejected burst must not pollute later trusted gravity.
        accel_time_=a.measured_us; accel_sequence_=a.sequence; accel_seen_=true;
    }
    Vec3 observed_down;
    if (usable && accel_seen_ && a.sequence == accel_sequence_ && a.measured_us == accel_time_) {
        const float raw_norm=norm(sample.accel.value), filtered_norm=norm(accel_);
        weight_=std::clamp(1-std::max(std::fabs(raw_norm/kGravity-1),std::fabs(filtered_norm/kGravity-1))/
                           config_.accel_norm_tolerance,0.0F,1.0F);
        if (!try_normalize(accel_*(-1),observed_down)) weight_=0;
    }
    if (!initialized_ && weight_ < 0.5F) return AttitudeError::WaitingForAccel;
    Vec3 gyro;
    if (!gyro_filter_.update(sample.gyro.value,dt_,config_.gyro_cutoff_hz,gyro)) return AttitudeError::Numerical;
    auto q=initialized_ ? state_.q_nb : tilt(observed_down);
    auto bias=initialized_ ? state_.gyro_bias_rad_s : Vec3{};
    if (initialized_) {
        const auto error=weight_ > 0 ? cross(observed_down,down_in_body(q))*weight_ : Vec3{};
        // Freeze residual-bias learning during weak gravity observations or rapid rotation.
        if (weight_ >= 0.5F && norm(gyro-bias) < 0.35F) {
            bias=bias-error*(config_.ki*dt_);
            const float magnitude=norm(bias);
            if (magnitude > config_.maximum_bias_rad_s) bias=bias*(config_.maximum_bias_rad_s/magnitude);
        }
        if (!propagate(q,gyro-bias+error*config_.kp,dt_,q)) return AttitudeError::Numerical;
    }
    if (!is_finite(bias) || !try_normalize(q,q)) return AttitudeError::Numerical;
    const auto rate=gyro-bias;
    const float yaw=yaw_of_unit(q);
    const float total=initialized_ ? state_.total_yaw_rad+std::remainder(yaw-yaw_rad_,2*kPi) : yaw;
    if (!std::isfinite(total)) return AttitudeError::Numerical;
    const auto epoch=state_.total_epoch+static_cast<unsigned>(!initialized_);
    state_={ {g.measured_us,now,g.sequence,true},q,rate,bias,g.measured_us,accel_time_,false,
             total,epoch };
    yaw_rad_=yaw;
    initialized_=true;
    return AttitudeError::None;
}
}
