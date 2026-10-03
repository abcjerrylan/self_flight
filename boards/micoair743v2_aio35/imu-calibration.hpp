#pragma once
#include "self_flight/core/data.hpp"

namespace self_flight::board {
constexpr std::uint32_t kCalibrationBoard = 0x74323535;
constexpr std::uint32_t kCalibrationSensor = 0xb0880f1e;
// This board's accepted six-face fit. Gyro bias is re-estimated each boot.
inline core::Calibration accelerometer_calibration() {
    core::Calibration c;
    c.board_identity = kCalibrationBoard;
    c.sensor_identity = kCalibrationSensor;
    c.accel_bias_m_s2 = {-0.019264221F, 0.074710846F, 0.011115074F};
    c.accel_scale = {1.000836850F, 0.998527408F, 1.002721430F};
    c.quality = core::CalibrationQuality::Accepted;
    return c;
}
}
