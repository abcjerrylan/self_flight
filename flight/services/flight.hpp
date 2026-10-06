#pragma once
#include "self_flight/core/flight_guard.hpp"
#include "dshot.hpp"
#include "sbus.hpp"

namespace self_flight::flight {
struct Snapshot {
    core::FlightStatus status{};
    dshot::Batch software_output{};
    std::uint32_t batches{}, stop_batches{}, rc_updates{}, control_reset_sequence{};
    sbus::Frame receiver{};
    sbus::Statistics receiver_stats{};
    bool receiver_mapping_ready{};
    core::PilotCommand pilot{};
};
// Receiver task publishes one complete decoded command, including failsafe.
void publish_pilot(const core::PilotCommand&);
// The UART consumer owns parsing/configuration; not called from an ISR.
bool configure_sbus(const sbus::ChannelMap&);
void receive_sbus_byte(std::uint8_t, core::TimestampUs received_us);
void reset_sbus();
// Only the gyro task owns the guard. P4A records frames; no hardware-output call exists.
void update(const core::AttitudeState&, const core::RateFeedback&, bool calibrated, core::TimestampUs now);
void deadline_missed(core::TimestampUs now);
Snapshot snapshot();
}
