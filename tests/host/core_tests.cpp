#include "self_flight/core/data.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

using namespace self_flight::core;
namespace {
int failures = 0;
int checks = 0;
void check(bool condition, const char* expression, int line) {
    ++checks;
    if (!condition) {
        ++failures;
        std::cerr << "line " << line << ": " << expression << '\n';
    }
}
#define CHECK(expression) check(static_cast<bool>(expression), #expression, __LINE__)
bool near(float a, float b, float tolerance = 2.0e-5F) {
    return std::isfinite(a) && std::isfinite(b) && std::fabs(a - b) <= tolerance;
}
bool near(Vec3 a, Vec3 b, float tolerance = 2.0e-5F) {
    return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) &&
           near(a.z, b.z, tolerance);
}
bool same(Quaternion a, Quaternion b) {
    return a.w == b.w && a.x == b.x && a.y == b.y && a.z == b.z;
}
Quaternion axis(Vec3 v, float radians) {
    Quaternion q{};
    CHECK(try_from_axis_angle(v, radians, q));
    return q;
}
Vec3 rotated(Quaternion q, Vec3 v) {
    Vec3 output{};
    CHECK(try_rotate(q, v, output));
    return output;
}
void vectors() {
    const Vec3 a{1.0F, 2.0F, 3.0F};
    CHECK(near(a + Vec3{2.0F, 1.0F, -1.0F}, {3.0F, 3.0F, 2.0F}));
    CHECK(near(a - Vec3{2.0F, 1.0F, -1.0F}, {-1.0F, 1.0F, 4.0F}));
    CHECK(near(a * 2.0F, {2.0F, 4.0F, 6.0F}));
    CHECK(near(dot(a, {2.0F, -1.0F, 1.0F}), 3.0F));
    CHECK(near(cross({1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}),
               {0.0F, 0.0F, 1.0F}));
    CHECK(near(cross(a, a), {}));
}
void identity() {
    CHECK(near(rotated({}, {1.0F, -2.0F, 3.0F}), {1.0F, -2.0F, 3.0F}));
    CHECK(near(rotated({}, {}), {}));
    const auto q = axis({1.0F, 0.0F, 0.0F}, 0.0F);
    CHECK(same(q, {}));
}
void known_axes() {
    const float half_pi = kPi * 0.5F;
    CHECK(near(rotated(axis({1.0F, 0.0F, 0.0F}, half_pi), {0.0F, 1.0F, 0.0F}),
               {0.0F, 0.0F, 1.0F}));
    CHECK(near(rotated(axis({0.0F, 1.0F, 0.0F}, half_pi), {0.0F, 0.0F, 1.0F}),
               {1.0F, 0.0F, 0.0F}));
    CHECK(near(rotated(axis({0.0F, 0.0F, 1.0F}, half_pi), {1.0F, 0.0F, 0.0F}),
               {0.0F, 1.0F, 0.0F}));
    CHECK(near(rotated(axis({0.0F, 0.0F, 1.0F}, -half_pi), {1.0F, 0.0F, 0.0F}),
               {0.0F, -1.0F, 0.0F}));
    CHECK(near(rotated(axis({1.0F, 0.0F, 0.0F}, kPi), {0.0F, 0.0F, -1.0F}),
               {0.0F, 0.0F, 1.0F}));
}
void composition() {
    const auto x = axis({1.0F, 0.0F, 0.0F}, kPi * 0.5F);
    const auto z = axis({0.0F, 0.0F, 1.0F}, kPi * 0.5F);
    const Vec3 input{0.0F, 1.0F, 0.0F};
    CHECK(near(rotated(multiply(z, x), input), {0.0F, 0.0F, 1.0F}));
    CHECK(near(rotated(multiply(x, z), input), {-1.0F, 0.0F, 0.0F}));
    CHECK(near(rotated(multiply(z, x), input), rotated(z, rotated(x, input))));
}
void inverse_sign() {
    const auto q = axis({1.0F, 2.0F, -3.0F}, 1.2F);
    const Vec3 v{2.0F, -1.0F, 0.3F};
    CHECK(near(rotated(conjugate(q), rotated(q, v)), v));
    CHECK(near(rotated({-q.w, -q.x, -q.y, -q.z}, v), rotated(q, v)));
    CHECK(near(rotated(multiply(q, conjugate(q)), v), v));
}
void normalization() {
    Vec3 v{};
    CHECK(try_normalize(Vec3{3.0F, 0.0F, 4.0F}, v));
    CHECK(near(v, {0.6F, 0.0F, 0.8F}));
    const float huge = std::numeric_limits<float>::max();
    CHECK(try_normalize(Vec3{huge, huge, huge}, v));
    CHECK(near(dot(v, v), 1.0F));
    // Combined norm exceeds the threshold despite each component being smaller.
    CHECK(try_normalize(Vec3{0.8e-6F, 0.8e-6F, 0.8e-6F}, v));
    CHECK(near(dot(v, v), 1.0F));
    CHECK(try_normalize(v, v)); // output may alias input
    Quaternion q{};
    CHECK(try_normalize(Quaternion{2.0F, 0.0F, 0.0F, 0.0F}, q));
    CHECK(same(q, {}));
    CHECK(try_normalize(Quaternion{huge, huge, huge, huge}, q));
    CHECK(near(q.w, 0.5F) && near(q.x, 0.5F) && near(q.y, 0.5F) && near(q.z, 0.5F));
    CHECK(try_normalize(q, q));
    CHECK(near(rotated({2.0F, 0.0F, 0.0F, 0.0F}, {1.0F, 2.0F, 3.0F}),
               {1.0F, 2.0F, 3.0F}));
}
void normalization_failures() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const Vec3 sentinel{7.0F, 8.0F, 9.0F};
    Vec3 v = sentinel;
    for (const Vec3 input : {Vec3{}, Vec3{1.0e-8F, 0.0F, 0.0F},
                            Vec3{nan, 0.0F, 0.0F}, Vec3{0.0F, inf, 0.0F},
                            Vec3{0.0F, 0.0F, -inf}, Vec3{kMinimumNorm, 0.0F, 0.0F}}) {
        CHECK(!try_normalize(input, v));
        CHECK(near(v, sentinel));
    }
    CHECK(!try_normalize(Vec3{1.0F, 0.0F, 0.0F}, v, -1.0F));
    CHECK(!try_normalize(Vec3{1.0F, 0.0F, 0.0F}, v, nan));
    CHECK(!try_normalize(Vec3{1.0F, 0.0F, 0.0F}, v, inf));
    CHECK(near(v, sentinel));
    const Quaternion qs{2.0F, 3.0F, 4.0F, 5.0F};
    Quaternion q = qs;
    for (const Quaternion input : {Quaternion{0.0F, 0.0F, 0.0F, 0.0F},
                                  Quaternion{1.0e-8F, 0.0F, 0.0F, 0.0F},
                                  Quaternion{nan, 0.0F, 0.0F, 0.0F},
                                  Quaternion{0.0F, inf, 0.0F, 0.0F},
                                  Quaternion{0.0F, 0.0F, inf, 0.0F},
                                  Quaternion{0.0F, 0.0F, 0.0F, -inf}}) {
        CHECK(!try_normalize(input, q));
        CHECK(same(q, qs));
    }
    CHECK(!try_normalize(Quaternion{}, q, -1.0F));
    CHECK(same(q, qs));
}
void axis_angle_failures() {
    const Quaternion sentinel{2.0F, 3.0F, 4.0F, 5.0F};
    Quaternion q = sentinel;
    CHECK(!try_from_axis_angle({}, 0.0F, q));
    CHECK(!try_from_axis_angle({1.0F, 0.0F, 0.0F},
                              std::numeric_limits<float>::quiet_NaN(), q));
    CHECK(!try_from_axis_angle({1.0F, 0.0F, 0.0F},
                              std::numeric_limits<float>::infinity(), q));
    CHECK(same(q, sentinel));
}
void rotation_failures() {
    const Vec3 sentinel{7.0F, 8.0F, 9.0F};
    Vec3 v = sentinel;
    CHECK(!try_rotate({0.0F, 0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, v));
    CHECK(!try_rotate({}, {0.0F, std::numeric_limits<float>::infinity(), 0.0F}, v));
    CHECK(near(v, sentinel));
    CHECK(try_rotate({}, v, v));
    CHECK(near(v, sentinel));
}
void time_intervals() {
    const auto dt = checked_delta(100000U, 101000U, 2000U);
    CHECK(dt.valid());
    CHECK(near(dt.seconds, 0.001F, 1.0e-8F));
    CHECK(checked_delta(0U, 1U, 1U).valid());
    const TimestampUs large = (TimestampUs{1} << 63U);
    CHECK(near(checked_delta(large, large + 1000U, 1000U).seconds, 0.001F, 1.0e-8F));
    CHECK(checked_delta(10U, 2010U, 2000U).valid());
}
void time_errors() {
    CHECK(checked_delta(1U, 1U, 100U).error == TimeError::Duplicate);
    CHECK(checked_delta(100U, 99U, 100U).error == TimeError::Backward);
    const auto gap = checked_delta(0U, 2001U, 2000U);
    CHECK(gap.error == TimeError::GapExceeded && gap.seconds == 0.0F);
    CHECK(checked_delta(0U, 1U, 0U).error == TimeError::InvalidLimit);
    const auto max = std::numeric_limits<TimestampUs>::max();
    CHECK(checked_delta(max, 0U, 1000U).error == TimeError::Backward);
    CHECK(!TimeDelta{}.valid());
}
void freshness() {
    CHECK(is_fresh(100U, 200U, 100U));
    CHECK(!is_fresh(100U, 201U, 100U));
    CHECK(!is_fresh(201U, 200U, 100U));
    CHECK(is_fresh(0U, 0U, 0U));
    CHECK(!is_fresh(std::numeric_limits<TimestampUs>::max(), 0U, 1000U));
}
void data_contracts() {
    CHECK(!is_usable(VectorSample{}, 0U, 1000U));
    VectorSample s{{100U, 150U, 3U, true}, {0.0F, 0.0F, -9.80665F}};
    CHECK(is_usable(s, 200U, 100U));
    CHECK(!is_usable(s, 201U, 100U));
    s.metadata.available_us = 99U;
    CHECK(!is_usable(s, 200U, 100U));
    s.metadata.available_us = 201U;
    CHECK(!is_usable(s, 200U, 100U));
    s.metadata.available_us = 150U;
    s.value.x = std::numeric_limits<float>::quiet_NaN();
    CHECK(!is_usable(s, 200U, 100U));
    CHECK(!ImuSample{}.gyro.metadata.valid && !ImuSample{}.accel.metadata.valid);
    CHECK(!AttitudeState{}.metadata.valid && !AttitudeState{}.absolute_yaw_valid);
    CHECK(!PilotCommand{}.metadata.valid && PilotCommand{}.receiver_failsafe);
    CHECK(!PilotCommand{}.arm_request);
    CHECK(Calibration{}.quality == CalibrationQuality::Unknown);
    CHECK(!ControlSetpoint{}.metadata.valid);
    CHECK(!ActuatorCommand{}.metadata.valid && !ActuatorCommand{}.output_allowed);
    for (const float output : ActuatorCommand{}.motor_normalized) { CHECK(output == 0.0F); }
    CHECK(FlightStatus{}.state == FlightState::Boot && !FlightStatus{}.output_allowed);
    ImuSample asynchronous{};
    asynchronous.gyro.metadata.measured_us = 1000U;
    asynchronous.accel.metadata.measured_us = 750U;
    CHECK(asynchronous.gyro.metadata.measured_us != asynchronous.accel.metadata.measured_us);
}
void rotation_properties() {
    // Deterministic coverage of varied axes/angles; no hardware simulation claim.
    for (int i = 1; i <= 128; ++i) {
        const float f = static_cast<float>(i);
        const auto q = axis({1.0F, f * 0.03F, -0.5F}, f * 0.07F);
        const auto p = axis({-0.2F, 1.0F, f * 0.02F}, -f * 0.03F);
        const Vec3 a{0.2F + f * 0.01F, -0.7F, 1.3F};
        const Vec3 b{-0.4F, 0.3F, 0.8F};
        const Vec3 ra = rotated(q, a);
        CHECK(near(dot(ra, ra), dot(a, a), 1.0e-4F));
        CHECK(near(dot(ra, rotated(q, b)), dot(a, b), 1.0e-4F));
        CHECK(near(rotated(conjugate(q), ra), a, 1.0e-4F));
        CHECK(near(rotated(multiply(p, q), a), rotated(p, ra), 1.0e-4F));
    }
}

struct Case { const char* name; void (*run)(); };
const Case cases[] = {
    {"vectors", vectors}, {"identity", identity}, {"known_axes", known_axes},
    {"composition", composition}, {"inverse_sign", inverse_sign},
    {"normalization", normalization}, {"normalization_failures", normalization_failures},
    {"axis_angle_failures", axis_angle_failures}, {"rotation_failures", rotation_failures},
    {"time_intervals", time_intervals}, {"time_errors", time_errors},
    {"freshness", freshness}, {"data_contracts", data_contracts},
    {"rotation_properties", rotation_properties}
};
} // namespace

int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "usage: core_tests <case>\n"; return 2; }
    for (const auto& test : cases) {
        if (std::string(argv[1]) == test.name) {
            test.run();
            std::cout << test.name << ": " << checks << " checks, " << failures << " failures\n";
            return failures == 0 ? 0 : 1;
        }
    }
    std::cerr << "unknown case\n";
    return 2;
}
