#pragma once
#include "bmi088.hpp"
#include "self_flight/core/imu.hpp"
#include "self_flight/core/calibration.hpp"
#include <cstdint>

namespace self_flight::imu {
struct Record {
    std::uint64_t measured_us{}, started_us{}, available_us{};
    std::uint32_t sequence{};
    bmi088::Raw raw{};
    bmi088::Sensor sensor{};
    bool valid{};
};
struct Statistics {
    std::uint32_t events{}, reads{}, missed{}, errors{}, overlaps{}, stale{};
    std::uint32_t maximum_latency_us{}, maximum_read_us{};
    std::uint32_t maximum_publication_latency_us{}, maximum_processing_us{}, publication_late{};
};
struct Temperature {
    float value_c{};
    std::uint64_t measured_us{};
    std::uint32_t errors{};
    bool valid{};
};
struct Snapshot {
    bmi088::Info info{};
    core::ImuSample sample{};
    Statistics stats[2]{};
    core::Calibration calibration{};
    Temperature temperature{};
    core::CalibrationError calibration_error{core::CalibrationError::Collecting};
    std::uint32_t calibration_samples[2]{}, calibration_restarts{};
    bool gyro_calibrated{}, accel_calibrated{};
    std::uint32_t dropped_logs{}, wait_timeouts{};
    bool initialized{};
};
unsigned start();
void drdy(bmi088::Sensor sensor);
Snapshot snapshot();
bool calibrated();
// Requires both calibrations. Each consumer owns its cursor; diagnostics do not consume it.
bool take_sample(core::ImuCursor& cursor, core::ImuSample& sample);
bool take_record(Record& record, unsigned wait_ticks);
void drop_logs(unsigned count);
}
