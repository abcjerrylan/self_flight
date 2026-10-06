#include "self_flight/core/attitude.hpp"
#include "self_flight/core/calibration.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace self_flight::core;
namespace {
int failures{};
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while(false)
bool near(float a,float b,float limit=0.0001F) { return std::fabs(a-b)<limit; }
float norm(Vec3 v) { return std::sqrt(dot(v,v)); }
float norm(Quaternion q) { return std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z); }
float agreement(Quaternion a,Quaternion b) { return std::fabs(a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z); }
VectorSample vector(Vec3 v,TimestampUs t,std::uint32_t seq) { return {{t,t+30,seq,true},v}; }
ImuSample frame(Vec3 gyro,Vec3 accel,TimestampUs t,std::uint32_t seq) {
    return {vector(gyro,t,seq),vector(accel,t,seq),true};
}
Quaternion axis(Vec3 direction,float angle) {
    Quaternion q; CHECK(try_from_axis_angle(direction,angle,q)); return q;
}
Vec3 force(Quaternion q) {
    Vec3 a; CHECK(try_rotate(conjugate(q),{0,0,-kGravity},a)); return a;
}
void filter() {
    LowPass3 f; Vec3 out;
    CHECK(f.update({},0,20,out));
    CHECK(f.update({1,0,0},0.001F,20,out));
    CHECK(near(out.x,0.001F/(0.001F+1/(2*kPi*20))));
    const float first=out.x;
    CHECK(f.update({1,0,0},0.002F,20,out) && out.x>first);
    const auto saved=out;
    CHECK(!f.update({NAN,0,0},0.001F,20,out) && out.x==saved.x);
    CHECK(!f.update({},0,20,out));
    CHECK(!f.update({},-1,20,out));
    CHECK(f.update({3,4,5},0.001F,0,out) && out.x==3);
    f.reset(); CHECK(f.update({7,8,9},0,20,out) && out.y==8);
}
void propagation() {
    for (const auto direction : {Vec3{1,0,0},Vec3{0,1,0},Vec3{0,0,1}}) {
        Quaternion q;
        for (unsigned i=0;i<1000;++i) CHECK(propagate(q,direction*(kPi/2),0.001F,q));
        CHECK(agreement(q,axis(direction,kPi/2))>0.99999F && near(norm(q),1));
    }
    const auto initial=axis({0,0,1},0.3F);
    auto q=initial;
    for (unsigned i=0;i<1000;++i) propagate(q,{kPi/2,0,0},0.001F,q);
    CHECK(agreement(q,multiply(initial,axis({1,0,0},kPi/2)))>0.99999F);
    Quaternion out{7,8,9,10};
    CHECK(!propagate({}, {},0,out) && out.w==7);
    CHECK(!propagate({0,0,0,0},{},0.001F,out) && out.w==7);
    CHECK(!propagate({}, {INFINITY,0,0},0.001F,out) && out.w==7);
    q={}; float angle=0;
    for (unsigned i=0;i<100000;++i) {
        const float dt=i%2 ? 0.0012F : 0.0008F;
        CHECK(propagate(q,{0,0,1},dt,q)); angle+=dt;
    }
    // Compare against elapsed double time, avoiding accumulated float time error.
    CHECK(agreement(q,axis({0,0,1},100))>0.99999F && near(norm(q),1));
}
void rotations() {
    for (const auto direction : {Vec3{1,0,0},Vec3{0,1,0},Vec3{0,0,1}}) {
        Mahony estimator;
        ImuSample latest;
        std::uint32_t a=0,g=0;
        for (TimestampUs t=0;t<=1000000;t+=250) {
            if (t%1250==0) latest.accel=vector(force(axis(direction,(kPi/2)*static_cast<float>(t)*1e-6F)),t,a++);
            if (t%1000==0) {
                latest.gyro=vector(direction*(kPi/2),t,g++);
                latest.accel_is_new=true;
                CHECK(estimator.update(latest,t+30)==AttitudeError::None);
            }
        }
        CHECK(agreement(estimator.state().q_nb,axis(direction,kPi/2))>0.9998F);
        CHECK(!estimator.state().absolute_yaw_valid && near(norm(estimator.state().q_nb),1));
    }
}
void tilt() {
    for (const auto q : {axis({1,0,0},0.4F),axis({0,1,0},-0.3F),axis({1,0,0},kPi)}) {
        Mahony estimator;
        CHECK(estimator.update(frame({},force(q),0,0),30)==AttitudeError::None);
        CHECK(agreement(estimator.state().q_nb,q)>0.99999F);
    }
    AttitudeConfig config; config.ki=0;
    Mahony estimator(config);
    estimator.update(frame({},{0,0,-kGravity},0,0),30);
    const auto target=axis({1,0,0},0.5F);
    for (unsigned n=1;n<=6000;++n)
        CHECK(estimator.update(frame({},force(target),n*1000,n),n*1000+30)==AttitudeError::None);
    CHECK(agreement(estimator.state().q_nb,target)>0.99999F);
    Vec3 euler; CHECK(attitude_euler(estimator.state().q_nb,euler) && near(euler.x,0.5F,0.001F));
    CHECK(near(estimator.state().total_yaw_rad,0));
}
void bias() {
    Mahony estimator;
    for (unsigned n=0;n<=120000;++n)
        CHECK(estimator.update(frame({0.02F,-0.015F,0},{0,0,-kGravity},n*1000,n),n*1000+30)==AttitudeError::None);
    CHECK(near(estimator.state().gyro_bias_rad_s.x,0.02F,0.0003F));
    CHECK(near(estimator.state().gyro_bias_rad_s.y,-0.015F,0.0003F));
    CHECK(norm(estimator.state().body_rate_rad_s)<0.0004F);
    Mahony yaw;
    for (unsigned n=0;n<=20000;++n) yaw.update(frame({0,0,0.01F},{0,0,-kGravity},n*1000,n),n*1000+30);
    Vec3 euler; attitude_euler(yaw.state().q_nb,euler);
    CHECK(near(euler.z,0.2F,0.001F) && near(yaw.state().gyro_bias_rad_s.z,0));
    CHECK(!yaw.state().absolute_yaw_valid);
    AttitudeConfig config; config.ki=100;
    Mahony bounded(config);
    bounded.update(frame({},{0,0,-kGravity},0,0),30);
    for (unsigned n=1;n<=1000;++n) bounded.update(frame({},force(axis({1,0,0},0.5F)),n*1000,n),n*1000+30);
    CHECK(norm(bounded.state().gyro_bias_rad_s)<=0.05001F);
}
void acceleration() {
    Mahony estimator;
    CHECK(estimator.update(frame({},{0,0,-2*kGravity},0,0),30)==AttitudeError::WaitingForAccel);
    CHECK(!estimator.state().metadata.valid);
    CHECK(estimator.update(frame({},{0,0,-kGravity},1000,1),1030)==AttitudeError::None);
    for (unsigned n=2;n<=1002;++n) {
        const auto accel=n%3==0 ? Vec3{NAN,0,0} : n%3==1 ? Vec3{} : Vec3{kGravity,0,-2*kGravity};
        CHECK(estimator.update(frame({0,0,0.2F},accel,n*1000,n),n*1000+30)==AttitudeError::None);
        CHECK(estimator.accel_weight()==0 && norm(estimator.state().gyro_bias_rad_s)==0);
    }
    Vec3 euler; attitude_euler(estimator.state().q_nb,euler);
    CHECK(near(euler.x,0) && near(euler.y,0) && near(euler.z,0.2F,0.003F));
    CHECK(estimator.update(frame({},{0,0,-kGravity},1003000,1003),1003030)==AttitudeError::None);
    CHECK(estimator.accel_weight()>0.99F);
}
void timing() {
    Mahony estimator;
    CHECK(estimator.update(frame({},{0,0,-kGravity},0,0xffffffffU),30)==AttitudeError::None);
    CHECK(estimator.update(frame({},{0,0,-kGravity},1000,0),1030)==AttitudeError::None);
    CHECK(estimator.update(frame({},{0,0,-kGravity},1000,1),1030)==AttitudeError::Timing);
    CHECK(estimator.update(frame({},{0,0,-kGravity},2000,1),2030)==AttitudeError::None);
    CHECK(estimator.update(frame({},{0,0,-kGravity},1000,2),2030)==AttitudeError::Timing);
    CHECK(estimator.update(frame({},{0,0,-kGravity},10000,2),10030)==AttitudeError::Timing);
    CHECK(!estimator.state().metadata.valid);
    CHECK(estimator.update(frame({},{0,0,-kGravity},11000,3),11030)==AttitudeError::None);
    CHECK(estimator.update(frame({},{0,0,-kGravity},12000,5),12030)==AttitudeError::Timing);
    CHECK(estimator.update(frame({},{0,0,-kGravity},13000,6),13030)==AttitudeError::None);
    auto stale=frame({},{0,0,-kGravity},14000,7);
    CHECK(estimator.update(stale,20000)==AttitudeError::InvalidSample);
    CHECK(estimator.update(stale,14010)==AttitudeError::InvalidSample);
    auto future=frame({},{0,0,-kGravity},14000,7);
    future.accel=vector({0,0,-kGravity},14500,7);
    CHECK(estimator.update(future,14530)==AttitudeError::None && estimator.accel_weight()==0);
}
void yaw_totals() {
    AttitudeConfig config; config.gyro_cutoff_hz=config.accel_cutoff_hz=config.kp=config.ki=0;
    for (const float sign : {-1.0F,1.0F}) {
        Mahony estimator(config);
        TimestampUs time=0;
        float previous=0;
        for (unsigned n=0;n<=4000;++n) {
            if (n) time+=n%2 ? 800 : 1200;
            CHECK(estimator.update(frame({0,0,sign*2*kPi},{0,0,-kGravity},time,n),time+30)==AttitudeError::None);
            const auto& s=estimator.state();
            CHECK(sign*(s.total_yaw_rad-previous)>=0 && std::fabs(s.total_yaw_rad-previous)<0.008F);
            previous=s.total_yaw_rad;
        }
        CHECK(near(previous,sign*8*kPi,0.003F) && estimator.state().total_epoch==1);
    }
    Mahony reverse(config);
    reverse.update(frame({},{0,0,-kGravity},0,0),30);
    for (unsigned n=1;n<=1000;++n)
        CHECK(reverse.update(frame({0,0,n<=500 ? 1.0F : -1.0F},{0,0,-kGravity},n*1000,n),n*1000+30)==AttitudeError::None);
    CHECK(near(reverse.state().total_yaw_rad,0));
}
void yaw_feedback() {
    // Tilted body Z rotation is not equal to Euler heading: follow the quaternion.
    AttitudeConfig config; config.gyro_cutoff_hz=config.accel_cutoff_hz=config.kp=config.ki=0;
    Mahony estimator(config);
    const auto initial=axis({0,1,0},0.7F);
    for (unsigned n=0;n<=1000;++n) {
        const auto target=multiply(initial,axis({0,0,1},n*0.001F));
        CHECK(estimator.update(frame({0,0,1},force(target),n*1000,n),n*1000+30)==AttitudeError::None);
        Vec3 euler; CHECK(attitude_euler(estimator.state().q_nb,euler));
        CHECK(near(estimator.state().total_yaw_rad,euler.z,0.0002F));
        CHECK(agreement(estimator.state().q_nb,target)>0.99999F);
    }
    CHECK(std::fabs(estimator.state().total_yaw_rad-1)>0.05F);
}
void yaw_state() {
    AttitudeConfig config; config.gyro_cutoff_hz=config.accel_cutoff_hz=config.kp=config.ki=0;
    Mahony estimator(config);
    CHECK(estimator.update(frame({},force(axis({1,0,0},0.4F)),0,0),30)==AttitudeError::None);
    CHECK(near(estimator.state().total_yaw_rad,0));
    CHECK(estimator.update(frame({0,0,1},{0,0,-kGravity},1000,1),1030)==AttitudeError::None);
    const auto saved=estimator.state().total_yaw_rad;
    CHECK(saved>0.0008F);
    CHECK(estimator.update(frame({NAN,0,0},{0,0,-kGravity},2000,2),2030)==AttitudeError::InvalidSample);
    CHECK(near(estimator.state().total_yaw_rad,saved) && estimator.state().total_epoch==1);
    CHECK(estimator.update(frame({0,0,1},{0,0,-kGravity},1000,2),1030)==AttitudeError::Timing);
    CHECK(near(estimator.state().total_yaw_rad,saved));
    CHECK(estimator.update(frame({0,0,1},{0,0,-kGravity},500,2),1030)==AttitudeError::Timing);
    CHECK(near(estimator.state().total_yaw_rad,saved));
    CHECK(estimator.update(frame({},{0,0,-kGravity},10000,2),10030)==AttitudeError::Timing);
    CHECK(near(estimator.state().total_yaw_rad,saved) && !estimator.state().metadata.valid);
    CHECK(estimator.update(frame({},{0,0,-kGravity},11000,3),11030)==AttitudeError::None);
    CHECK(near(estimator.state().total_yaw_rad,0) && estimator.state().total_epoch==2);
}
void failures_test() {
    AttitudeConfig config; config.kp=NAN;
    Mahony invalid(config);
    CHECK(invalid.update(frame({},{0,0,-kGravity},0,0),30)==AttitudeError::Config);
    Mahony estimator;
    estimator.update(frame({},{0,0,-kGravity},0,0),30);
    const auto q=estimator.state().q_nb;
    CHECK(estimator.update(frame({NAN,0,0},{0,0,-kGravity},1000,1),1030)==AttitudeError::InvalidSample);
    CHECK(agreement(q,estimator.state().q_nb)>0.99999F && !estimator.state().metadata.valid);
    CHECK(estimator.update(frame({},{0,0,-kGravity},1000,1),1030)==AttitudeError::None);
    Vec3 out{7,8,9}; CHECK(!attitude_euler({0,0,0,0},out) && out.x==7);
}
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string test=argv[1];
    if (test=="filter") filter();
    else if (test=="propagation") propagation();
    else if (test=="rotations") rotations();
    else if (test=="tilt") tilt();
    else if (test=="bias") bias();
    else if (test=="acceleration") acceleration();
    else if (test=="timing") timing();
    else if (test=="yaw_totals") yaw_totals();
    else if (test=="yaw_state") yaw_state();
    else if (test=="yaw_feedback") yaw_feedback();
    else if (test=="failures") failures_test();
    else return 2;
    return failures ? 1 : 0;
}
