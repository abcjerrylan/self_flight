#include "dshot.hpp"
#include <cmath>
#include <iostream>
#include <string>

using namespace self_flight;
namespace {
int failures{};
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while(false)
bool checksum_valid(unsigned frame) {
    unsigned parity=0;
    for (unsigned bit=0;bit<4;++bit) {
        const auto column=((frame>>bit) ^ (frame>>(bit+4)) ^ (frame>>(bit+8)) ^ (frame>>(bit+12))) & 1;
        parity |= column<<bit;
    }
    return parity==0;
}
void encoding() {
    std::uint16_t frame=7;
    CHECK(dshot::encode(0,false,frame) && frame==0);
    CHECK(dshot::encode(48,false,frame) && frame==0x0606);
    CHECK(dshot::encode(2047,false,frame) && frame==0xffee);
    CHECK(dshot::encode(2047,true,frame) && frame==0xffff);
    for (unsigned value=0;value<=2047;++value) {
        if (value>0 && value<48) continue;
        for (bool telemetry : {false,true}) {
            CHECK(dshot::encode(static_cast<std::uint16_t>(value),telemetry,frame));
            CHECK(static_cast<unsigned>(frame>>5)==value && ((frame>>4)&1)==static_cast<unsigned>(telemetry));
            CHECK(checksum_valid(frame));
            for (unsigned bit=0;bit<16;++bit) CHECK(!checksum_valid(frame^(1U<<bit)));
        }
    }
    for (unsigned value : {1U,47U,2048U,65535U}) {
        frame=7; CHECK(!dshot::encode(static_cast<std::uint16_t>(value),false,frame) && frame==7);
    }
}
core::FlightStatus status() {
    core::FlightStatus s; s.metadata={1000,1000,1,true}; s.state=core::FlightState::Armed; s.output_allowed=true; return s;
}
core::ActuatorCommand command() {
    core::ActuatorCommand c; c.metadata={1000,1000,1,true}; c.output_allowed=true; c.motor_normalized={0,.1F,.5F,1}; return c;
}
void mapping() {
    const auto b=dshot::prepare(command(),status(),1000);
    CHECK(b.stop_reasons==0);
    CHECK((b.frames[0]>>5)==0 && (b.frames[1]>>5)==248 && (b.frames[2]>>5)==1048 && (b.frames[3]>>5)==2047);
    for (auto f : b.frames) CHECK(checksum_valid(f) && !(f&16));
}
void gates() {
    for (unsigned n=0;n<11;++n) {
        auto c=command(); auto s=status(); core::TimestampUs now=1000,age=3000;
        if (n==0) c.output_allowed=false;
        if (n==1) s.output_allowed=false;
        if (n==2) s.state=core::FlightState::Disarmed;
        if (n==3) c.metadata.valid=false;
        if (n==4) s.metadata.valid=false;
        if (n==5) c.metadata.available_us=1001;
        if (n==6) now=4001;
        if (n==7) c.motor_normalized[2]=NAN;
        if (n==8) c.motor_normalized[3]=1.01F;
        if (n==9) c.motor_normalized[0]=-.01F;
        if (n==10) age=0;
        const auto b=dshot::prepare(c,s,now,age);
        CHECK(b.stop_reasons!=0);
        for (auto f : b.frames) CHECK(f==0);
    }
    CHECK(dshot::prepare(command(),status(),4000).stop_reasons==0);
    CHECK(dshot::prepare(core::ActuatorCommand{},core::FlightStatus{},0).stop_reasons!=0);
}
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string name=argv[1];
    if (name=="encoding") encoding(); else if (name=="mapping") mapping(); else if (name=="gates") gates(); else return 2;
    return failures ? 1 : 0;
}
