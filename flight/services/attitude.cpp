#include "attitude.hpp"
#include "flight.hpp"
#include "imu.hpp"
#include "platform.hpp"
#include "tx_api.h"
#include <algorithm>

namespace self_flight::attitude {
namespace {
TX_THREAD thread;
TX_EVENT_FLAGS_GROUP events;
alignas(8) unsigned char stack[4096];
Snapshot state;
void run(ULONG) {
    core::ImuCursor cursor;
    core::ImuPipeline estimator;
    bool seen=false;
    std::uint32_t sequence=0;
    for (;;) {
        ULONG flags{};
        if (tx_event_flags_get(&events,1,TX_OR_CLEAR,&flags,5) != TX_SUCCESS) {
            Snapshot failed;
            {
                const platform::CriticalSection lock;
                ++state.timeouts;
                state.estimate.metadata.valid=state.rate.metadata.valid=state.rate.derivative_valid=false;
                failed=state;
            }
            flight::update(failed.estimate,failed.rate,imu::calibrated(),platform::time_us());
            continue;
        }
        core::ImuSample sample;
        if (!imu::take_sample(cursor,sample)) {
            Snapshot failed;
            {
                const platform::CriticalSection lock;
                state.estimate.metadata.valid=state.rate.metadata.valid=state.rate.derivative_valid=false;
                if (seen) { ++state.rejections; state.error=core::AttitudeError::InvalidSample; }
                failed=state;
            }
            flight::update(failed.estimate,failed.rate,imu::calibrated(),platform::time_us());
            continue;
        }
        const auto started=platform::time_us();
        const auto error=estimator.update(sample,started);
        auto estimate=estimator.state();
        auto rate=estimator.rate();
        const auto solved=platform::time_us();
        estimate.metadata.available_us=rate.metadata.available_us=solved;
        estimate.metadata.valid &= solved-sample.gyro.metadata.measured_us < 1000;
        rate.metadata.valid &= estimate.metadata.valid;
        rate.derivative_valid &= rate.metadata.valid;
        flight::update(estimate,rate,imu::calibrated(),solved);
        const auto finished=platform::time_us();
        const auto runtime=static_cast<std::uint32_t>(finished-started);
        const auto latency=static_cast<std::uint32_t>(finished-sample.gyro.metadata.measured_us);
        if (latency >= 1000) flight::deadline_missed(finished);
        estimate.metadata.available_us=finished;
        estimate.metadata.valid &= latency < 1000;
        rate.metadata.available_us=finished;
        rate.metadata.valid &= latency < 1000;
        rate.derivative_valid &= rate.metadata.valid;
        const auto missed=seen ? sample.gyro.metadata.sequence-sequence-1 : 0;
        seen=true; sequence=sample.gyro.metadata.sequence;
        {
            const platform::CriticalSection lock;
            state.estimate=estimate; state.rate=rate; state.error=error; state.rate_error=estimator.rate_error();
            state.accel_weight=estimator.accel_weight(); state.dt_s=estimator.dt_s();
            state.updates += error == core::AttitudeError::None;
            state.rejections += error != core::AttitudeError::None;
            state.timing_errors += error == core::AttitudeError::Timing;
            state.missed += missed; state.late += latency >= 1000;
            state.runtime_us=runtime; state.latency_us=latency;
            state.maximum_runtime_us=std::max(state.maximum_runtime_us,runtime);
            state.maximum_latency_us=std::max(state.maximum_latency_us,latency);
        }
    }
}
}
unsigned start() {
    const auto status=tx_event_flags_create(&events,const_cast<char*>("attitude_ready"));
    if (status != TX_SUCCESS) return status;
    return tx_thread_create(&thread,const_cast<char*>("attitude"),run,0,stack,sizeof(stack),
                            6,6,TX_NO_TIME_SLICE,TX_AUTO_START);
}
void notify() { tx_event_flags_set(&events,1,TX_OR); }
Snapshot snapshot() {
    Snapshot copy;
    { const platform::CriticalSection lock; copy=state; }
    if (!core::is_fresh(copy.estimate.metadata.measured_us,platform::time_us(),5000))
        copy.estimate.metadata.valid=false;
    if (!core::is_fresh(copy.rate.metadata.measured_us,platform::time_us(),5000))
        copy.rate.metadata.valid=copy.rate.derivative_valid=false;
    return copy;
}
}
