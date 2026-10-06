#include "self_flight/core/flight_guard.hpp"
#include "dshot.hpp"
#include <cmath>
#include <iostream>
#include <string>

using namespace self_flight::core;
namespace {
int failures{};
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while(false)
struct Rig {
    FlightGuard guard;
    PilotCommand pilot;
    FlightHealth health;
    TimestampUs now{};
    Rig() {
        pilot.receiver_failsafe=false;
        health.calibration_accepted=health.controller_ready=health.output_driver_ready=true;
        health.rate.derivative_valid=true;
        health.attitude.total_epoch=1;
    }
    void sensors() {
        health.attitude.metadata=health.rate.metadata={now,now,static_cast<std::uint32_t>(now/1000),true};
    }
    const FlightStatus& tick(bool refresh_rc=true, bool refresh_sensors=true, TimestampUs dt=1000) {
        now+=dt;
        if (refresh_rc) pilot.metadata={now,now,static_cast<std::uint32_t>(now/1000),true};
        if (refresh_sensors) sensors();
        return guard.update(pilot,health,now);
    }
    void ready() { tick(); CHECK(tick().state==FlightState::Disarmed); }
    void arm() { ready(); pilot.arm_request=true; CHECK(tick().state==FlightState::Armed); }
};
void arming() {
    Rig r; CHECK(!r.guard.status().output_allowed);
    r.pilot.arm_request=true; r.ready();
    CHECK(r.tick().state==FlightState::Disarmed); // No automatic arm after boot.
    r.pilot.arm_request=false; r.tick();
    r.pilot.arm_request=true; CHECK(r.tick().output_allowed && r.guard.reset_sequence()==1);
    r.pilot.collective=.8F; CHECK(r.tick().output_allowed); // Arming threshold does not cap flight throttle.
    r.pilot.arm_request=false; CHECK(r.tick().state==FlightState::Disarmed && !r.guard.status().output_allowed);
    CHECK(r.guard.reset_sequence()==2);
    Rig high; high.ready(); high.pilot.collective=.2F; high.pilot.arm_request=true;
    CHECK(high.tick().arm_block_reasons & ThrottleHigh);
    high.pilot.collective=0; CHECK(!high.tick().output_allowed);
    high.pilot.arm_request=false; high.tick(); high.pilot.arm_request=true; CHECK(high.tick().output_allowed);
    Rig late_rc; late_rc.pilot.receiver_failsafe=true; late_rc.ready();
    late_rc.pilot.receiver_failsafe=false; late_rc.pilot.arm_request=true;
    CHECK(!late_rc.tick().output_allowed); // First RC frame HIGH is not an observed rising edge.
    late_rc.pilot.arm_request=false; late_rc.tick(); late_rc.pilot.arm_request=true;
    CHECK(late_rc.tick().output_allowed);
}
void calibration() {
    Rig r; r.health.calibration_accepted=false;
    CHECK(r.tick().state==FlightState::Calibrating);
    r.pilot.arm_request=true; CHECK(r.tick().state==FlightState::Calibrating);
    r.health.calibration_accepted=true; CHECK(r.tick().state==FlightState::Disarmed);
    CHECK(!r.tick().output_allowed);
    r.pilot.arm_request=false; r.tick(); r.pilot.arm_request=true; CHECK(r.tick().output_allowed);
}
void loss_recovery() {
    Rig r; r.arm(); r.pilot.receiver_failsafe=true;
    CHECK(r.tick().state==FlightState::Fault && (r.guard.status().fault_reasons & ReceiverInvalid));
    CHECK(r.guard.reset_sequence()==2);
    r.pilot.receiver_failsafe=false; CHECK(r.tick().state==FlightState::Fault);
    r.pilot.arm_request=false; r.pilot.collective=.3F; CHECK(r.tick().state==FlightState::Fault);
    r.pilot.collective=0; CHECK(r.tick().state==FlightState::Disarmed);
    CHECK(!r.tick().output_allowed); r.pilot.arm_request=true; CHECK(r.tick().output_allowed);
    Rig stale; stale.arm();
    for (unsigned n=0;n<100;++n) CHECK(stale.tick(false).state==FlightState::Armed);
    CHECK(stale.tick(false).state==FlightState::Fault);
}
void health() {
    for (unsigned fault=0;fault<7;++fault) {
        Rig r; r.arm();
        if (fault==0) r.health.calibration_accepted=false;
        if (fault==1) r.health.controller_ready=false;
        if (fault==2) r.health.output_driver_ready=false;
        if (fault==3) r.health.rate.derivative_valid=false;
        if (fault==4) r.health.rate.angular_accel_rad_s2.x=NAN;
        if (fault==5) r.health.attitude.q_nb={0,0,0,0};
        if (fault==6) r.health.attitude.total_epoch++;
        CHECK(r.tick().state==FlightState::Fault && !r.guard.status().output_allowed);
        if (fault==6) CHECK(r.guard.status().fault_reasons & ReferenceChanged);
    }
    Rig stale; stale.arm();
    for (unsigned n=0;n<5;++n) CHECK(stale.tick(true,false).output_allowed);
    CHECK(stale.tick(true,false).state==FlightState::Fault);
    Rig emergency; emergency.arm(); emergency.pilot.emergency_stop=true; emergency.pilot.receiver_failsafe=true;
    CHECK(emergency.tick().fault_reasons & EmergencyStop);
}
void timing() {
    Rig r; r.arm(); CHECK(r.tick(true,true,6000).fault_reasons & GuardTiming);
    CHECK(!r.guard.status().metadata.valid);
    r.pilot.arm_request=false; CHECK(r.tick().state==FlightState::Disarmed);
    r.pilot.arm_request=true; CHECK(r.tick().state==FlightState::Armed);
    CHECK(r.guard.update(r.pilot,r.health,r.now).state==FlightState::Fault);
    CHECK(!r.guard.status().metadata.valid);
    r.pilot.arm_request=false; CHECK(r.tick().state==FlightState::Disarmed);
    r.pilot.arm_request=true; CHECK(r.tick().state==FlightState::Armed);
    CHECK(r.guard.update(r.pilot,r.health,r.now-1).fault_reasons & GuardTiming);
    FlightGuard bad({0,5000,5000,.05F});
    CHECK(bad.update(r.pilot,r.health,r.now).arm_block_reasons & InvalidGuardConfig);
}
void rc_contract() {
    Rig r; r.tick(); CHECK(is_usable(r.pilot,r.now,100000));
    for (unsigned n=0;n<7;++n) {
        auto p=r.pilot;
        if (n==0) p.roll=NAN;
        if (n==1) p.pitch=1.01F;
        if (n==2) p.collective=-.1F;
        if (n==3) p.receiver_failsafe=true;
        if (n==4) p.mode=static_cast<FlightMode>(99);
        if (n==5) p.metadata.available_us=r.now+1;
        if (n==6) p.metadata.valid=false;
        CHECK(!is_usable(p,r.now,100000));
    }
}
void deadline() {
    Rig r; r.arm(); r.now+=1500;
    r.guard.deadline_missed(r.now);
    CHECK(r.guard.status().state==FlightState::Fault && !r.guard.status().output_allowed);
    CHECK((r.guard.status().fault_reasons & ControlDeadline) && r.guard.reset_sequence()==2);
    CHECK(!r.guard.status().metadata.valid);
    CHECK(r.tick().state==FlightState::Fault);
    r.pilot.arm_request=false; CHECK(r.tick().state==FlightState::Disarmed);
    r.pilot.arm_request=true; CHECK(r.tick().output_allowed);
}
void dry_chain() {
    Rig r; r.arm();
    ActuatorCommand command; command.metadata={r.now,r.now,1,true}; command.output_allowed=true;
    command.motor_normalized={.1F,.2F,.3F,.4F};
    auto batch=self_flight::dshot::prepare(command,r.guard.status(),r.now);
    CHECK(batch.stop_reasons==0 && batch.frames[0]!=0 && batch.frames[3]!=0);
    r.health.attitude.total_epoch++; r.tick();
    batch=self_flight::dshot::prepare(command,r.guard.status(),r.now);
    CHECK(batch.stop_reasons & self_flight::dshot::Permission);
    for (auto frame : batch.frames) CHECK(frame==0);
    Rig unavailable; unavailable.health.controller_ready=unavailable.health.output_driver_ready=false;
    unavailable.ready(); unavailable.pilot.arm_request=true;
    CHECK(!unavailable.tick().output_allowed); // Same policy as the P4A MCU dry service.
}
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string name=argv[1];
    if (name=="arming") arming(); else if (name=="calibration") calibration();
    else if (name=="loss_recovery") loss_recovery(); else if (name=="health") health();
    else if (name=="timing") timing(); else if (name=="rc_contract") rc_contract();
    else if (name=="dry_chain") dry_chain(); else if (name=="deadline") deadline();
    else return 2;
    return failures ? 1 : 0;
}
