#include "self_flight/core/calibration.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace self_flight::core;
namespace {
int failures{};
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while(false)
bool near(float a,float b) { return std::fabs(a-b) < 0.0001F; }
VectorSample sample(Vec3 value, std::uint64_t time, std::uint32_t sequence, bool valid=true) {
    return {{time,time+30,sequence,valid},value};
}
void fill(StationaryCalibration& window, std::uint64_t offset=0, std::uint32_t initial=0) {
    std::uint32_t a=initial,g=initial;
    for (std::uint64_t t=0; t<=3010000; t+=250) {
        if (t%1000 == 0) CHECK(window.add(true,sample({0.01F,-0.002F,0.003F},offset+t,g++)) == CalibrationError::None);
        if (t%1250 == 0) CHECK(window.add(false,sample({0,0,kGravity},offset+t,a++)) == CalibrationError::None);
    }
}
void stationary() {
    StationaryCalibration window;
    GyroEstimate result;
    CHECK(!window.estimate(result));
    fill(window, (std::uint64_t{1}<<32)-1000000, 0xfffffff0);
    CHECK(window.estimate(result));
    CHECK(near(result.bias.x,0.01F) && near(result.bias.y,-0.002F));
    CHECK(result.variance.x == 0 && result.measured_us > (std::uint64_t{1}<<32));
}
void rejection() {
    StationaryCalibration window;
    fill(window);
    CHECK(window.add(true,sample({0.3F,0,0},3011000,3011)) == CalibrationError::Motion);
    CHECK(window.count(false)==0 && window.count(true)==0);
    fill(window);
    CHECK(window.add(true,sample({},3011000,3011,false)) == CalibrationError::InvalidSample);
    fill(window);
    CHECK(window.add(true,sample({},3012000,3012)) == CalibrationError::Timing);
    window.reset();
    CHECK(window.add(true,sample({},100,1)) == CalibrationError::None);
    CHECK(window.add(true,sample({},100,2)) == CalibrationError::Timing);
    CHECK(window.add(true,sample({std::numeric_limits<float>::quiet_NaN(),0,0},0,1)) == CalibrationError::InvalidSample);
    // A steady rate above the bias bound is rejected even with zero variance.
    for (unsigned i=0;i<30;++i) window.add(true,sample({0.08F,0,0},i*1000,i));
    CHECK(window.count(true)==0);
    for (unsigned i=0;i<30;++i) window.add(false,sample({i%2 ? 0.5F:-0.5F,0,kGravity},i*1250,i));
    CHECK(window.count(false)==0);
}
void late_motion() {
    // Repeat across bucket boundaries: a quiet prefix must not hide recent motion.
    for (const auto start : {2800000U,2825000U,2900000U}) {
        StationaryCalibration window;
        bool rejected = false, accepted = false;
        std::uint32_t a=0,g=0;
        for (std::uint64_t t=0; t<=3010000; t+=250) {
            if (t%1000 == 0) rejected |= window.add(true,
                sample({0,0,t >= start ? 0.1F : 0.0F},t,g++)) == CalibrationError::Motion;
            if (t%1250 == 0) window.add(false,sample({0,0,kGravity},t,a++));
            GyroEstimate out;
            accepted |= window.estimate(out);
        }
        CHECK(rejected && !accepted);
        // Stop motion without restarting the collector or breaking either sequence.
        for (std::uint64_t t=3010250; t<=6200000; t+=250) {
            if (t%1000 == 0) {
                const auto error=window.add(true,sample({},t,g++));
                CHECK(error == CalibrationError::None || error == CalibrationError::Motion);
            }
            if (t%1250 == 0) window.add(false,sample({0,0,kGravity},t,a++));
        }
        GyroEstimate out;
        CHECK(window.estimate(out) && near(out.bias.z,0));
    }
    // Norm stays near g while a late tilt changes its direction.
    StationaryCalibration window;
    bool rejected = false;
    std::uint32_t a=0,g=0;
    for (std::uint64_t t=0; t<=3010000; t+=250) {
        if (t%1000 == 0) window.add(true,sample({},t,g++));
        if (t%1250 == 0) rejected |= window.add(false,
            sample(t >= 2800000 ? Vec3{1,0,9.75553F} : Vec3{0,0,kGravity},t,a++)) == CalibrationError::Motion;
    }
    GyroEstimate out;
    CHECK(rejected && !window.estimate(out));
}
std::array<Vec3,6> faces() {
    // Independent synthetic biases (0.1,-0.2,0.05), scales (1.02,0.98,1.01).
    return {{{kGravity/1.02F+0.1F,-0.2F,0.05F},{-kGravity/1.02F+0.1F,-0.2F,0.05F},
             {0.1F,kGravity/0.98F-0.2F,0.05F},{0.1F,-kGravity/0.98F-0.2F,0.05F},
             {0.1F,-0.2F,kGravity/1.01F+0.05F},{0.1F,-0.2F,-kGravity/1.01F+0.05F}}};
}
void fit() {
    Calibration result;
    CHECK(fit_accelerometer(faces(),result) == CalibrationError::None);
    CHECK(near(result.accel_bias_m_s2.x,0.1F) && near(result.accel_bias_m_s2.y,-0.2F));
    CHECK(near(result.accel_scale.x,1.02F) && near(result.accel_scale.y,0.98F));
    auto bad=faces(); bad[0]=bad[1];
    const auto saved=result.accel_scale;
    CHECK(fit_accelerometer(bad,result) == CalibrationError::Face);
    CHECK(result.accel_scale.x==saved.x);
    bad=faces(); bad[2].z=2;
    CHECK(fit_accelerometer(bad,result) == CalibrationError::Face);
    bad=faces(); bad[5].z=std::numeric_limits<float>::infinity();
    CHECK(fit_accelerometer(bad,result) == CalibrationError::InvalidSample);
}
void application() {
    Calibration c; fit_accelerometer(faces(),c);
    Vec3 out{7,8,9};
    CHECK(!apply_calibration({},false,c,out) && out.x==7);
    c.quality=CalibrationQuality::Accepted; c.gyro_bias_rad_s={0.01F,0,0};
    CHECK(apply_calibration(faces()[0],false,c,out));
    CHECK(near(out.x,kGravity) && near(out.y,0) && near(out.z,0));
    CHECK(apply_calibration({0.01F,0,0},true,c,out) && near(out.x,0));
    c.accel_scale.z=0;
    CHECK(!apply_calibration({},false,c,out));
    c.accel_scale.z=1; c.temperature_valid=true; c.temperature_c=100;
    CHECK(!calibration_parameters_valid(c));
}
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string test=argv[1];
    if (test=="stationary") stationary();
    else if (test=="rejection") rejection();
    else if (test=="late_motion") late_motion();
    else if (test=="fit") fit();
    else if (test=="application") application();
    else return 2;
    return failures ? 1 : 0;
}
