#include "sbus.hpp"
#include "self_flight/core/flight_guard.hpp"
#include <iostream>
#include <string>

using namespace self_flight;
namespace {
int failures{};
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while(false)
std::array<std::uint8_t,25> pack(const std::array<std::uint16_t,16>& channels, unsigned flags=0, unsigned footer=0) {
    std::array<std::uint8_t,25> bytes{}; bytes[0]=15;
    for (unsigned c=0;c<16;++c) for (unsigned bit=0;bit<11;++bit)
        bytes[1+(c*11+bit)/8] |= ((channels[c]>>bit)&1)<<((c*11+bit)%8);
    bytes[23]=static_cast<std::uint8_t>(flags); bytes[24]=static_cast<std::uint8_t>(footer);
    return bytes;
}
bool send(sbus::Parser& p,const std::array<std::uint8_t,25>& bytes,core::TimestampUs start,sbus::Frame& out) {
    bool completed=false;
    for (unsigned n=0;n<bytes.size();++n) completed=p.push(bytes[n],start+n*120,out) || completed;
    return completed;
}
void decoding() {
    sbus::Parser p; sbus::Frame out;
    std::array<std::uint16_t,16> channels{};
    for (unsigned round=0;round<100;++round) {
        for (unsigned c=0;c<16;++c) channels[c]=static_cast<std::uint16_t>((round*227+c*131)&2047);
        CHECK(send(p,pack(channels),round*10000,out));
        CHECK(out.metadata.valid && out.metadata.measured_us==round*10000+2880);
        CHECK(out.channels==channels && out.metadata.sequence==round+1);
    }
    channels.fill(2047); CHECK(send(p,pack(channels),1000000,out) && out.channels==channels);
    for (unsigned flag=0;flag<16;++flag) {
        CHECK(send(p,pack(channels,flag),1010000+flag*10000,out));
        CHECK(out.digital17==static_cast<bool>(flag&1) && out.digital18==static_cast<bool>(flag&2));
        CHECK(out.frame_lost==static_cast<bool>(flag&4) && out.failsafe==static_cast<bool>(flag&8));
    }
    CHECK(p.stats().lost==8 && p.stats().failsafe==8);
    for (unsigned footer : {0U,4U,0x14U,0x24U,0x34U})
        CHECK(send(p,pack(channels,0,footer),1300000+footer*10000,out));
}
void recovery() {
    sbus::Parser p; sbus::Frame out; out.metadata.sequence=99;
    std::array<std::uint16_t,16> channels{}; channels.fill(992);
    const auto good=pack(channels);
    CHECK(!p.push(7,0,out));
    for (unsigned n=0;n<10;++n) CHECK(!p.push(good[n],100+n*120,out));
    CHECK(send(p,good,10000,out)); // A partial frame expires at an inter-frame gap.
    CHECK(p.stats().partial_resets==1);
    const auto saved=out;
    CHECK(!send(p,pack(channels,0,0x55),20000,out) && out.metadata.sequence==saved.metadata.sequence);
    CHECK(!send(p,pack(channels,0x80),30000,out));
    CHECK(send(p,good,40000,out));
    CHECK(p.stats().malformed==2);
    p.reset();
    CHECK(send(p,good,50000,out));
    // Truncate a 150Hz frame; the following frame's 3.79ms gap restores alignment.
    const auto resets=p.stats().partial_resets;
    for (unsigned n=0;n<25;++n) if (n!=12) CHECK(!p.push(good[n],56667+n*120,out));
    CHECK(send(p,good,63334,out) && out.channels==channels);
    CHECK(p.stats().partial_resets==resets+1);
}
void timing() {
    sbus::Parser p; sbus::Frame out;
    const auto bytes=pack({});
    CHECK(!p.push(15,10000,out));
    CHECK(!p.push(0,9999,out) && p.stats().time_errors==1);
    CHECK(send(p,bytes,20000,out));
    p.reset(); bool completed=false;
    for (auto byte : bytes) completed=p.push(byte,30000,out) || completed;
    CHECK(completed); // A timestamped DMA chunk can have equal per-byte times.
    p.reset();
    CHECK(send(p,bytes,40000,out) && out.metadata.sequence==3);
}
sbus::ChannelMap mapping() { return {0,1,3,2,4,5,6,172,992,1811,{}}; }
void mapping_test() {
    sbus::Frame f; f.metadata={1000,1000,1,true}; f.channels.fill(992);
    const auto m=mapping(); core::PilotCommand p;
    CHECK(sbus::map_pilot(f,m,p) && p.roll==0 && p.pitch==0 && !p.arm_request && !p.receiver_failsafe);
    f.channels[0]=172; f.channels[1]=1811; f.channels[2]=172; f.channels[3]=1811;
    f.channels[4]=1811; f.channels[5]=1811; f.channels[6]=1811;
    CHECK(sbus::map_pilot(f,m,p) && p.roll==-1 && p.pitch==1 && p.yaw==1 && p.collective==0);
    CHECK(p.arm_request && p.emergency_stop && p.mode==core::FlightMode::Attitude);
    auto reversed=m; reversed.reversed={true,true,true};
    CHECK(sbus::map_pilot(f,reversed,p) && p.roll==1 && p.pitch==-1 && p.yaw==-1);
    f.channels[2]=2047; CHECK(sbus::map_pilot(f,m,p) && p.collective==1);
    const auto previous=p; f.frame_lost=true;
    CHECK(!sbus::map_pilot(f,m,p) && p.metadata.sequence==previous.metadata.sequence);
    f.failsafe=true; CHECK(sbus::map_pilot(f,m,p) && p.receiver_failsafe);
    f.frame_lost=f.failsafe=false;
    CHECK(!sbus::map_pilot(f,sbus::ChannelMap{},p));
    auto bad=m; bad.arm=bad.roll; CHECK(!sbus::valid_mapping(bad));
    bad=m; bad.center=bad.low; CHECK(!sbus::valid_mapping(bad));
    bad=m; bad.throttle=16; CHECK(!sbus::valid_mapping(bad));
    bad=m; bad.emergency_stop=16; CHECK(!sbus::valid_mapping(bad));
    bad=m; bad.high=2048; CHECK(!sbus::valid_mapping(bad));
    auto optional=m; optional.attitude_mode=optional.emergency_stop=sbus::kUnassigned;
    CHECK(sbus::map_pilot(f,optional,p) && p.mode==core::FlightMode::Rate && !p.emergency_stop);
    f.metadata.valid=false; CHECK(!sbus::map_pilot(f,m,p));
}
void lost_policy() {
    core::FlightGuard guard; core::FlightHealth h;
    h.calibration_accepted=h.controller_ready=h.output_driver_ready=true; h.rate.derivative_valid=true;
    h.attitude.total_epoch=1;
    sbus::Frame frame; frame.channels.fill(992); frame.channels[2]=172;
    core::PilotCommand pilot;
    for (unsigned n=0;n<62;++n) {
        const core::TimestampUs t=n*1000;
        if (n%20==0) { // Actual 50Hz receiver versus 1kHz gyro loop.
            frame.metadata={t,t,n/20,true}; frame.channels[4]=n==0 ? 172 : 1811;
            frame.frame_lost=n>=40; frame.failsafe=n>=60;
            const bool mapped=sbus::map_pilot(frame,mapping(),pilot);
            CHECK(mapped==(n!=40));
            if (n==40) CHECK(pilot.metadata.measured_us==20000);
        }
        h.attitude.metadata=h.rate.metadata={t,t,n,true};
        const auto state=guard.update(pilot,h,t).state;
        if (n>=2 && n<20) CHECK(state==core::FlightState::Disarmed);
        if (n>=20 && n<60) CHECK(state==core::FlightState::Armed);
        if (n>=60) CHECK(state==core::FlightState::Fault);
    }
}
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string name=argv[1];
    if (name=="decoding") decoding(); else if (name=="recovery") recovery();
    else if (name=="timing") timing(); else if (name=="mapping") mapping_test(); else if (name=="lost_policy") lost_policy(); else return 2;
    return failures ? 1 : 0;
}
