#pragma once
#include "self_flight/core/imu_pipeline.hpp"

namespace self_flight::attitude {
struct Snapshot {
    core::AttitudeState estimate{};
    core::RateFeedback rate{};
    core::AttitudeError error{core::AttitudeError::WaitingForAccel};
    core::RateError rate_error{core::RateError::None};
    float accel_weight{}, dt_s{};
    std::uint32_t updates{}, rejections{}, timing_errors{}, missed{}, timeouts{}, late{};
    std::uint32_t runtime_us{}, latency_us{}, maximum_runtime_us{}, maximum_latency_us{};
};
unsigned start();
void notify(); // Sampling thread, after gyro publication; not a second SPI reader.
Snapshot snapshot();
}
