#include "self_flight/core/math.hpp"

#include <algorithm>
#include <cmath>

namespace self_flight::core {

Vec3 operator+(Vec3 a, Vec3 b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
Vec3 operator-(Vec3 a, Vec3 b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
Vec3 operator*(Vec3 v, float s) noexcept { return {v.x * s, v.y * s, v.z * s}; }
float dot(Vec3 a, Vec3 b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
bool is_finite(Vec3 v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool is_finite(Quaternion q) noexcept {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z);
}
Quaternion multiply(Quaternion a, Quaternion b) noexcept {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Quaternion conjugate(Quaternion q) noexcept { return {q.w, -q.x, -q.y, -q.z}; }

bool try_normalize(Vec3 input, Vec3& out, float minimum_norm) noexcept {
    if (!is_finite(input) || !std::isfinite(minimum_norm) || minimum_norm < 0.0F) {
        return false;
    }
    const float scale = std::max({std::fabs(input.x), std::fabs(input.y),
                                  std::fabs(input.z)});
    if (scale == 0.0F) { return false; }
    const Vec3 scaled{input.x / scale, input.y / scale, input.z / scale};
    const float norm = std::sqrt(dot(scaled, scaled));
    // Compare without forming scale*norm, which may overflow for finite input.
    if (scale <= minimum_norm / norm) { return false; }
    const Vec3 result{scaled.x / norm, scaled.y / norm, scaled.z / norm};
    if (!is_finite(result)) { return false; }
    out = result;
    return true;
}

bool try_normalize(Quaternion input, Quaternion& out, float minimum_norm) noexcept {
    if (!is_finite(input) || !std::isfinite(minimum_norm) || minimum_norm < 0.0F) {
        return false;
    }
    const float scale = std::max({std::fabs(input.w), std::fabs(input.x),
                                  std::fabs(input.y), std::fabs(input.z)});
    if (scale == 0.0F) { return false; }
    const Quaternion s{input.w / scale, input.x / scale, input.y / scale,
                       input.z / scale};
    const float norm = std::sqrt(s.w * s.w + s.x * s.x + s.y * s.y + s.z * s.z);
    if (scale <= minimum_norm / norm) { return false; }
    const Quaternion result{s.w / norm, s.x / norm, s.y / norm, s.z / norm};
    if (!is_finite(result)) { return false; }
    out = result;
    return true;
}

bool try_from_axis_angle(Vec3 axis, float angle_rad, Quaternion& out) noexcept {
    Vec3 unit{};
    if (!std::isfinite(angle_rad) || !try_normalize(axis, unit)) { return false; }
    const float half = angle_rad * 0.5F;
    const float sine = std::sin(half);
    const Quaternion candidate{std::cos(half), unit.x * sine, unit.y * sine,
                               unit.z * sine};
    return try_normalize(candidate, out);
}

bool try_rotate(Quaternion q, Vec3 v, Vec3& out) noexcept {
    Quaternion unit{};
    if (!is_finite(v) || !try_normalize(q, unit)) { return false; }
    const Vec3 u{unit.x, unit.y, unit.z};
    const Vec3 twice_cross = cross(u, v) * 2.0F;
    const Vec3 result = v + twice_cross * unit.w + cross(u, twice_cross);
    if (!is_finite(result)) { return false; }
    out = result;
    return true;
}

} // namespace self_flight::core
