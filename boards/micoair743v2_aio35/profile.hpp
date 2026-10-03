#pragma once
#include "self_flight/core/math.hpp"

namespace self_flight::board {
// ArduPilot's official MicoAir743v2 hwdef: Rz(270deg)*Rx(180deg).
// AIO mounting and each axis still require physical verification before flight.
inline core::Vec3 sensor_to_body(core::Vec3 v) { return {-v.y, -v.x, -v.z}; }
}
