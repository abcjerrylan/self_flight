#include "self_flight/core/imu_pipeline.hpp"
#include "self_flight/core/calibration.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace self_flight::core;
namespace {
int failures{};
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while(false)
bool near(float a,float b,float e=0.0001F) { return std::fabs(a-b)<e; }
VectorSample frame(Vec3 v,TimestampUs t,std::uint32_t s) { return {{t,t+30,s,true},v}; }
void ramp() {
    RateFeedbackFilter f({0,0,3000,5000}); RateFeedback r;
    TimestampUs t=0;
    for (std::uint32_t n=0;n<1000;++n) {
        const float seconds=static_cast<float>(t)*1e-6F;
        CHECK(f.update(frame({2*seconds,-3*seconds,seconds},t,n),{},t+30,r)==RateError::None);
        CHECK(r.metadata.valid && r.derivative_valid==(n!=0));
        if (n) CHECK(near(r.angular_accel_rad_s2.x,2,.001F) && near(r.angular_accel_rad_s2.y,-3,.001F));
        t += n%2 ? 1200 : 800;
    }
}
void bias() {
    RateFeedbackFilter f; RateFeedback r;
    for (unsigned n=0;n<20;++n) {
        const auto b=n<10 ? Vec3{} : Vec3{.02F,-.01F,.03F};
        CHECK(f.update(frame({.1F,.2F,.3F},n*1000,n),b,n*1000+30,r)==RateError::None);
        CHECK(near(r.body_rate_rad_s.x,.1F-b.x));
        CHECK(dot(r.angular_accel_rad_s2,r.angular_accel_rad_s2)==0);
    }
}
void filters() {
    RateFeedbackFilter f; RateFeedback r;
    CHECK(f.update(frame({},0,0),{},30,r)==RateError::None);
    CHECK(f.update(frame({1,0,0},1000,1),{},1030,r)==RateError::None);
    const float alpha=.001F/(.001F+1/(2*kPi*80));
    const float d_alpha=.001F/(.001F+1/(2*kPi*30));
    CHECK(near(r.filtered_gyro_rad_s.x,alpha));
    CHECK(near(r.angular_accel_rad_s2.x,alpha*1000*d_alpha,.001F));
    RateFeedbackFilter filtered({0,30,3000,5000}), raw({0,0,3000,5000});
    RateFeedback a,b; double sum_a=0,sum_b=0;
    for (unsigned n=0;n<3000;++n) {
        const auto sample=frame({std::sin(2*kPi*100*n*.001F),0,0},n*1000,n);
        CHECK(filtered.update(sample,{},n*1000+30,a)==RateError::None);
        CHECK(raw.update(sample,{},n*1000+30,b)==RateError::None);
        if (n>1000) { sum_a+=a.angular_accel_rad_s2.x*a.angular_accel_rad_s2.x; sum_b+=b.angular_accel_rad_s2.x*b.angular_accel_rad_s2.x; }
    }
    const double ratio=std::sqrt(sum_a/sum_b);
    CHECK(ratio>.2 && ratio<.4);
    std::cout<<"Synthetic 100Hz derivative RMS ratio (30Hz LP / bypass)="<<ratio<<'\n';
}
void timing() {
    RateFeedbackFilter f; RateFeedback r;
    CHECK(f.update(frame({1,0,0},1000,1),{},1030,r)==RateError::None);
    CHECK(f.update(frame({1,0,0},2000,2),{},2030,r)==RateError::None);
    const auto saved=r;
    CHECK(f.update(frame({},2000,2),{},2030,r)==RateError::Timing && r.metadata.sequence==saved.metadata.sequence);
    CHECK(f.update(frame({},1500,1),{},2030,r)==RateError::Timing);
    CHECK(f.update(frame({},4000,4),{},4030,r)==RateError::Timing);
    CHECK(f.update(frame({2,0,0},5000,5),{},5030,r)==RateError::None && !r.derivative_valid);
    CHECK(f.update(frame({2,0,0},10000,6),{},10030,r)==RateError::Timing);
    CHECK(f.update(frame({2,0,0},11000,7),{},11030,r)==RateError::None && !r.derivative_valid);
    f.reset();
    CHECK(f.update(frame({},0,UINT32_MAX),{},30,r)==RateError::None);
    CHECK(f.update(frame({},1000,0),{},1030,r)==RateError::None && r.derivative_valid);
}
void failures_test() {
    RateFeedbackFilter f; RateFeedback r; r.metadata.sequence=99;
    CHECK(f.update(frame({NAN,0,0},0,0),{},30,r)==RateError::InvalidSample && r.metadata.sequence==99);
    CHECK(f.update(frame({},0,0),{NAN,0,0},30,r)==RateError::InvalidSample);
    auto s=frame({},0,0); s.metadata.available_us=40;
    CHECK(f.update(s,{},30,r)==RateError::InvalidSample);
    s=frame({},0,0); CHECK(f.update(s,{},6000,r)==RateError::InvalidSample);
    RateFeedbackFilter bad({NAN,30,3000,5000});
    CHECK(bad.update(frame({},0,0),{},30,r)==RateError::Config);
    RateFeedbackFilter overflow({0,0,3000,5000});
    const float big=std::numeric_limits<float>::max();
    CHECK(overflow.update(frame({big,0,0},0,0),{-big,0,0},30,r)==RateError::Numerical);
    CHECK(overflow.update(frame({},0,0),{},30,r)==RateError::None && !r.derivative_valid);
    CHECK(overflow.update(frame({big,0,0},1000,1),{},1030,r)==RateError::Numerical);
    CHECK(overflow.update(frame({},1000,1),{},1030,r)==RateError::None && r.derivative_valid);
}
void pipeline() {
    ImuPipeline p; Mahony original;
    for (unsigned n=0;n<3000;++n) {
        const ImuSample s{frame({.1F*std::sin(n*.01F),.02F,.2F},n*1000,n),
                          frame({0,0,-kGravity},n*1000,n),true};
        CHECK(p.update(s,n*1000+30)==AttitudeError::None);
        CHECK(original.update(s,n*1000+30)==AttitudeError::None);
        const auto a=p.state().q_nb,b=original.state().q_nb;
        CHECK(near(a.w,b.w,1e-6F) && near(a.x,b.x,1e-6F) && near(a.y,b.y,1e-6F) && near(a.z,b.z,1e-6F));
        CHECK(near(p.rate().body_rate_rad_s.x,p.state().body_rate_rad_s.x,1e-6F));
        CHECK(p.rate().derivative_valid==(n!=0));
    }
    const auto epoch=p.state().total_epoch;
    const ImuSample missed{frame({},3001000,3001),frame({0,0,-kGravity},3001000,3001),true};
    CHECK(p.update(missed,3001030)==AttitudeError::Timing && !p.state().metadata.valid && !p.rate().metadata.valid);
    const ImuSample next{frame({},3002000,3002),frame({0,0,-kGravity},3002000,3002),true};
    CHECK(p.update(next,3002030)==AttitudeError::None && p.state().total_epoch==epoch+1);
    CHECK(!p.rate().derivative_valid);
    ImuPipeline waiting;
    const ImuSample no_gravity{frame({},0,0),frame({},0,0),true};
    CHECK(waiting.update(no_gravity,30)==AttitudeError::WaitingForAccel && !waiting.rate().metadata.valid);
}
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string name=argv[1];
    if (name=="ramp") ramp(); else if (name=="bias") bias(); else if (name=="filters") filters();
    else if (name=="timing") timing(); else if (name=="failures") failures_test(); else if (name=="pipeline") pipeline();
    else return 2;
    return failures ? 1 : 0;
}
