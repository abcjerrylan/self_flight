#include "imu.hpp"
#include "platform.hpp"
#include "profile.hpp"
#include "imu-calibration.hpp"
#include <cmath>
#include "tx_api.h"
#include <algorithm>

namespace self_flight::imu {
namespace {
TX_THREAD thread;
TX_EVENT_FLAGS_GROUP events;
TX_QUEUE logs;
alignas(8) unsigned char stack[4096];
static_assert(sizeof(ULONG) == 4 && sizeof(Record) % sizeof(ULONG) == 0);
static_assert(sizeof(Record) <= 16 * sizeof(ULONG));
alignas(8) unsigned char log_storage[256 * sizeof(Record)];
struct Pending { std::uint64_t time{}; std::uint32_t sequence{}; };
Pending pending[2];
Snapshot state;
bool capturing = false;

Pending latest(unsigned index) {
    const platform::CriticalSection lock;
    return pending[index];
}

void run(ULONG) {
    bmi088::Driver driver(platform::imu_bus());
    const auto info = driver.initialize();
    {
        const platform::CriticalSection lock;
        state.info = info;
        state.initialized = info.error == bmi088::Error::None;
        capturing = state.initialized;
    }
    if (!capturing) return;
    std::uint32_t consumed[2]{};
    std::uint32_t accel_in_gyro_frame = 0;
    core::StationaryCalibration stationary;
    auto calibration = board::accelerometer_calibration();
    const bool accel_ready = core::calibration_parameters_valid(calibration) &&
        calibration.quality == core::CalibrationQuality::Accepted &&
        calibration.board_identity == board::kCalibrationBoard &&
        calibration.sensor_identity == board::kCalibrationSensor;
    calibration.board_identity = board::kCalibrationBoard;
    calibration.sensor_identity = board::kCalibrationSensor;
    calibration.quality = core::CalibrationQuality::Unknown;
    if (!accel_ready) { calibration.accel_bias_m_s2 = {}; calibration.accel_scale = {1,1,1}; }
    bool gyro_ready = false;
    for (;;) {
        ULONG flags{};
        if (tx_event_flags_get(&events, 3, TX_OR_CLEAR, &flags, 10) != TX_SUCCESS) {
            const platform::CriticalSection lock;
            ++state.wait_timeouts;
        }
        for (const unsigned index : {1U, 0U}) {
            const auto event = latest(index);
            if (event.sequence == consumed[index]) continue;
            Record record{};
            record.sensor = static_cast<bmi088::Sensor>(index);
            record.sequence = event.sequence;
            record.measured_us = event.time;
            record.started_us = platform::time_us();
            const bool ok = driver.read(record.sensor, record.raw);
            record.available_us = platform::time_us();
            const bool overlap = latest(index).sequence != event.sequence;
            const auto latency = static_cast<std::uint32_t>(record.available_us - event.time);
            const auto duration = static_cast<std::uint32_t>(record.available_us - record.started_us);
            const bool stale = latency >= (index == 1 ? 1000U : 1250U);
            record.valid = ok && !overlap && !stale;
            const auto sensor_si = bmi088::to_si(record.raw, record.sensor);
            core::CalibrationError cal_error = gyro_ready ? core::CalibrationError::None : core::CalibrationError::Collecting;
            if (!gyro_ready) {
                const auto error = stationary.add(index == 1,
                    {{event.time,record.available_us,event.sequence,record.valid},sensor_si});
                if (error != core::CalibrationError::None) cal_error = error;
                core::GyroEstimate estimate;
                if (stationary.estimate(estimate)) {
                    calibration.gyro_bias_rad_s = estimate.bias;
                    calibration.stationary_gyro_variance = std::max({estimate.variance.x,estimate.variance.y,estimate.variance.z});
                    calibration.measured_us = estimate.measured_us;
                    calibration.temperature_valid = driver.read_temperature(calibration.temperature_c);
                    if (calibration.temperature_valid && (!std::isfinite(calibration.temperature_c) ||
                        calibration.temperature_c < -40 || calibration.temperature_c > 85)) calibration.temperature_valid = false;
                    calibration.quality = accel_ready ? core::CalibrationQuality::Accepted : core::CalibrationQuality::Unknown;
                    gyro_ready = true;
                    cal_error = core::CalibrationError::None;
                }
            }
            auto corrected = sensor_si;
            if (index == 1 && gyro_ready) corrected = sensor_si-calibration.gyro_bias_rad_s;
            if (index == 0 && accel_ready) {
                auto accel_only = calibration;
                accel_only.quality = core::CalibrationQuality::Accepted;
                if (!core::apply_calibration(sensor_si,false,accel_only,corrected)) record.valid = false;
            }
            const auto corrected_available_us = platform::time_us();
            const bool corrected_valid = record.valid && corrected_available_us-event.time < (index == 1 ? 1000U : 1250U);
            {
                const platform::CriticalSection lock;
                state.calibration = calibration;
                state.gyro_calibrated = gyro_ready;
                state.accel_calibrated = accel_ready;
                state.calibration_error = cal_error;
                state.calibration_samples[0] = stationary.count(false);
                state.calibration_samples[1] = stationary.count(true);
                state.calibration_restarts += cal_error != core::CalibrationError::None && cal_error != core::CalibrationError::Collecting;
                auto& stats = state.stats[index];
                stats.events = event.sequence;
                stats.missed += event.sequence - consumed[index] - 1;
                ++stats.reads;
                stats.errors += !ok;
                stats.overlaps += overlap;
                stats.stale += stale;
                stats.maximum_latency_us = std::max(stats.maximum_latency_us, latency);
                stats.maximum_read_us = std::max(stats.maximum_read_us, duration);
                auto& sample = index == 1 ? state.sample.gyro : state.sample.accel;
                sample.metadata = {event.time, corrected_available_us, event.sequence, corrected_valid};
                sample.value = board::sensor_to_body(corrected);
                if (index == 1) {
                    state.sample.accel_is_new = state.sample.accel.metadata.valid &&
                        state.sample.accel.metadata.sequence != accel_in_gyro_frame;
                    accel_in_gyro_frame = state.sample.accel.metadata.sequence;
                }
            }
            consumed[index] = event.sequence;
            if (ok && tx_queue_send(&logs, &record, TX_NO_WAIT) != TX_SUCCESS) drop_logs(1);
        }
    }
}
}

unsigned start() {
    auto status = tx_event_flags_create(&events, const_cast<char*>("imu_drdy"));
    if (status != TX_SUCCESS) return status;
    status = tx_queue_create(&logs, const_cast<char*>("imu_logs"), sizeof(Record) / sizeof(ULONG),
                             log_storage, sizeof(log_storage));
    if (status != TX_SUCCESS) return status;
    return tx_thread_create(&thread, const_cast<char*>("imu"), run, 0, stack, sizeof(stack),
                             5, 5, TX_NO_TIME_SLICE, TX_AUTO_START);
}

void drdy(bmi088::Sensor sensor) {
    const unsigned index = static_cast<unsigned>(sensor);
    {
        const platform::CriticalSection lock;
        if (!capturing) return;
        pending[index].time = platform::time_us();
        ++pending[index].sequence;
    }
    tx_event_flags_set(&events, 1U << index, TX_OR);
}

Snapshot snapshot() {
    Snapshot copy;
    {
        const platform::CriticalSection lock;
        copy = state;
        copy.stats[0].events = pending[0].sequence;
        copy.stats[1].events = pending[1].sequence;
    }
    const auto now = platform::time_us();
    if (!core::is_usable(copy.sample.gyro, now, 5000)) copy.sample.gyro.metadata.valid = false;
    if (!core::is_usable(copy.sample.accel, now, 5000)) copy.sample.accel.metadata.valid = false;
    return copy;
}

bool take_record(Record& record, unsigned wait_ticks) {
    return tx_queue_receive(&logs, &record, wait_ticks) == TX_SUCCESS;
}
void drop_logs(unsigned count) {
    const platform::CriticalSection lock;
    state.dropped_logs += count;
}
}
