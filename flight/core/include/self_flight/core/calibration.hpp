#pragma once
#include "self_flight/core/data.hpp"
#include <array>

namespace self_flight::core {
constexpr float kGravity = 9.80665F;
enum class CalibrationError : std::uint8_t { None, Collecting, InvalidSample, Timing, Motion, Face, Parameters };

struct VectorMoments {
    std::uint32_t count{};
    Vec3 mean{}, m2{};
    void add(Vec3 value);
    Vec3 variance() const;
};
struct GyroEstimate {
    Vec3 bias{}, variance{}, accel_mean{};
    TimestampUs measured_us{};
};
// Sensor SI axes. Invalid input or motion discards the entire candidate window.
class StationaryCalibration {
public:
    CalibrationError add(bool gyro, const VectorSample& sample);
    bool estimate(GyroEstimate& out) const;
    void reset();
    std::uint32_t count(bool gyro) const { return moments_[gyro ? 1 : 0].count; }
private:
    VectorMoments moments_[2]{}, recent_[2]{}, previous_[2]{};
    TimestampUs recent_first_[2]{};
    SampleMetadata last_[2]{};
    TimestampUs first_[2]{};
};

// Means in sensor axes, ordered +X,-X,+Y,-Y,+Z,-Z. Diagonal bias/scale only.
CalibrationError fit_accelerometer(const std::array<Vec3, 6>& means, Calibration& out);
bool calibration_parameters_valid(const Calibration& calibration);
bool apply_calibration(Vec3 input, bool gyro, const Calibration& calibration, Vec3& out);
}
