#pragma once

#include "self_flight/core/math.hpp"
#include "self_flight/core/time.hpp"

#include <array>
#include <cstdint>

namespace self_flight::core {

// measured_us: DRDY-based measurement-time estimate, not read completion.
struct SampleMetadata {
    TimestampUs measured_us{0U};
    TimestampUs available_us{0U};
    std::uint32_t sequence{0U};
    bool valid{false};
};

struct VectorSample {
    SampleMetadata metadata{};
    Vec3 value{};
};

inline bool is_usable(const VectorSample& sample, TimestampUs now,
                       TimestampUs maximum_age_us) noexcept {
    const auto& meta = sample.metadata;
    return meta.valid && is_finite(sample.value) &&
           meta.measured_us <= meta.available_us && meta.available_us <= now &&
           is_fresh(meta.measured_us, now, maximum_age_us);
}

// Values are body FRD: gyro rad/s; accelerometer specific force m/s^2.
struct ImuSample {
    VectorSample gyro{};
    VectorSample accel{};
    bool accel_is_new{false};
};

enum class CalibrationQuality : std::uint8_t { Unknown, Rejected, Accepted };
// Biases and scales are in sensor axes, applied before mounting rotation.
struct Calibration {
    std::uint32_t format_version{1U};
    std::uint32_t board_identity{0U};
    std::uint32_t sensor_identity{0U};
    Vec3 gyro_bias_rad_s{};
    Vec3 accel_bias_m_s2{};
    Vec3 accel_scale{1.0F, 1.0F, 1.0F};
    float temperature_c{0.0F};
    float stationary_gyro_variance{0.0F};
    TimestampUs measured_us{0U};
    CalibrationQuality quality{CalibrationQuality::Unknown};
    bool temperature_valid{false};
};

struct AttitudeState {
    SampleMetadata metadata{};
    Quaternion q_nb{};
    Vec3 body_rate_rad_s{};
    Vec3 gyro_bias_rad_s{};
    TimestampUs gyro_measured_us{0U};
    TimestampUs accel_measured_us{0U};
    bool absolute_yaw_valid{false};
};

enum class FlightMode : std::uint8_t { Rate, Attitude };
struct PilotCommand {
    SampleMetadata metadata{}; // measured_us is the valid RC control-frame time.
    float roll{0.0F};          // normalized [-1,1], not an angle/rate yet
    float pitch{0.0F};
    float yaw{0.0F};
    float collective{0.0F};    // [0,1]
    FlightMode mode{FlightMode::Rate};
    bool arm_request{false};
    bool emergency_stop{false};
    bool receiver_failsafe{true};
};

struct ControlSetpoint {
    SampleMetadata metadata{};
    FlightMode mode{FlightMode::Rate};
    float roll_rad{0.0F};
    float pitch_rad{0.0F};
    Vec3 body_rate_rad_s{};
    float collective{0.0F};
};

struct ActuatorCommand {
    SampleMetadata metadata{};
    std::array<float, 4> motor_normalized{}; // electrical M1..M4, not arm positions
    std::array<bool, 4> saturated{};
    bool output_allowed{false};
};

enum class FlightState : std::uint8_t { Boot, Calibrating, Disarmed, Armed, Fault };
struct FlightStatus {
    SampleMetadata metadata{};
    FlightState state{FlightState::Boot};
    std::uint32_t arm_block_reasons{0U};
    std::uint32_t fault_reasons{0U}; // bit definitions belong to the P4 state machine
    std::uint32_t missed_sample_events{0U};
    std::uint32_t deadline_misses{0U};
    std::uint32_t dropped_logs{0U};
    bool output_allowed{false};
};

} // namespace self_flight::core
