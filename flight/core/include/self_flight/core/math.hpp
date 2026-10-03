#pragma once

namespace self_flight::core {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kMinimumNorm = 1.0e-6F;

struct Vec3 {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

// Hamilton WXYZ. q_ab maps a vector in frame b into frame a.
struct Quaternion {
    float w{1.0F};
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
};

Vec3 operator+(Vec3 a, Vec3 b) noexcept;
Vec3 operator-(Vec3 a, Vec3 b) noexcept;
Vec3 operator*(Vec3 v, float scalar) noexcept;
float dot(Vec3 a, Vec3 b) noexcept;
Vec3 cross(Vec3 a, Vec3 b) noexcept;
bool is_finite(Vec3 v) noexcept;
bool is_finite(Quaternion q) noexcept;

// Raw arithmetic may overflow: callers must check before publishing results.
Quaternion multiply(Quaternion q_ab, Quaternion q_bc) noexcept;
Quaternion conjugate(Quaternion q) noexcept;

// Norm must be strictly greater than minimum_norm. Failure never changes out.
bool try_normalize(Vec3 input, Vec3& out,
                   float minimum_norm = kMinimumNorm) noexcept;
bool try_normalize(Quaternion input, Quaternion& out,
                   float minimum_norm = kMinimumNorm) noexcept;
bool try_from_axis_angle(Vec3 axis, float angle_rad, Quaternion& out) noexcept;
// Locally normalizes q; rejects invalid q/v and non-finite rotated output.
bool try_rotate(Quaternion q, Vec3 v, Vec3& out) noexcept;

} // namespace self_flight::core
