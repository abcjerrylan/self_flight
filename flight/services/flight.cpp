#include "flight.hpp"
#include "platform.hpp"

namespace self_flight::flight {
namespace {
core::FlightGuard guard;
core::PilotCommand pilot;
Snapshot state;
sbus::Parser receiver;
sbus::ChannelMap mapping;
}
bool configure_sbus(const sbus::ChannelMap& config) {
    if (!sbus::valid_mapping(config)) return false;
    mapping=config;
    receiver.reset();
    const platform::CriticalSection lock;
    pilot={}; state.receiver_mapping_ready=true;
    return true;
}
void reset_sbus() {
    receiver.reset();
    const platform::CriticalSection lock;
    state.receiver.metadata.valid=false;
}
void receive_sbus_byte(std::uint8_t byte, core::TimestampUs received_us) {
    sbus::Frame frame;
    const bool complete=receiver.push(byte,received_us,frame);
    if (complete) {
        core::PilotCommand command;
        if (sbus::map_pilot(frame,mapping,command)) publish_pilot(command);
    }
    const platform::CriticalSection lock;
    state.receiver_stats=receiver.stats();
    if (complete) state.receiver=frame;
}
void publish_pilot(const core::PilotCommand& command) {
    const platform::CriticalSection lock;
    pilot=command;
    ++state.rc_updates;
}
void update(const core::AttitudeState& attitude, const core::RateFeedback& rate,
            bool calibrated, core::TimestampUs now) {
    core::PilotCommand command;
    { const platform::CriticalSection lock; command=pilot; }
    const core::FlightHealth health{attitude,rate,calibrated,false,false};
    const auto status=guard.update(command,health,now);
    // PID/mixer and timer-DMA are not implemented. Both readiness flags stay false.
    const auto output=dshot::prepare(core::ActuatorCommand{},status,now);
    const platform::CriticalSection lock;
    state.status=status; state.software_output=output;
    state.control_reset_sequence=guard.reset_sequence();
    ++state.batches;
    bool stopped=true;
    for (auto frame : output.frames) stopped &= frame == 0;
    state.stop_batches += stopped;
}
Snapshot snapshot() {
    Snapshot copy;
    { const platform::CriticalSection lock; copy=state; copy.pilot=pilot; }
    if (!core::is_fresh(copy.status.metadata.measured_us,platform::time_us(),5000)) {
        copy.status.metadata.valid=false; copy.status.output_allowed=false;
    }
    return copy;
}
void deadline_missed(core::TimestampUs now) {
    guard.deadline_missed(now);
    const auto output=dshot::prepare(core::ActuatorCommand{},guard.status(),now);
    const platform::CriticalSection lock;
    state.status=guard.status(); state.software_output=output;
    state.control_reset_sequence=guard.reset_sequence();
}
}
