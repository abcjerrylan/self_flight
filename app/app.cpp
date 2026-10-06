#include "app.h"
#include "platform.hpp"
#include "imu.hpp"
#include "attitude.hpp"
#include "flight.hpp"
#include "rc.hpp"
#include "tx_api.h"
#include <cstdio>
#include <cmath>
#include <cstring>

namespace {
TX_THREAD log_thread;
alignas(8) unsigned char stack[4096];

void run(ULONG) {
    using namespace self_flight;
    ULONG last_status = tx_time_get(), last_attitude = last_status;
    char batch[512]{};
    unsigned used = 0, count = 0;
    auto flush = [&] {
        if (used && !platform::write(batch)) imu::drop_logs(count);
        used = count = 0;
        batch[0] = '\0';
    };
    for (;;) {
        imu::Record record;
        if (imu::take_record(record, used ? TX_NO_WAIT : 4)) {
            char line[160];
            // Decimal microseconds via seconds/remainder: nano printf need not support %llu.
            const auto length = std::snprintf(line, sizeof(line),
                "IMU,%c,%lu,%lu%06lu,%lu%06lu,%lu%06lu,%d,%d,%d,%lu,%u\r\n",
                record.sensor == bmi088::Sensor::Accel ? 'A' : 'G',
                static_cast<unsigned long>(record.sequence),
                static_cast<unsigned long>(record.measured_us / 1000000), static_cast<unsigned long>(record.measured_us % 1000000),
                static_cast<unsigned long>(record.started_us / 1000000), static_cast<unsigned long>(record.started_us % 1000000),
                static_cast<unsigned long>(record.available_us / 1000000), static_cast<unsigned long>(record.available_us % 1000000),
                record.raw.x, record.raw.y, record.raw.z, static_cast<unsigned long>(record.raw.sensor_time), record.valid);
            if (length <= 0 || static_cast<unsigned>(length) >= sizeof(line)) {
                imu::drop_logs(1);
                continue;
            }
            if (used + static_cast<unsigned>(length) >= sizeof(batch)) flush();
            for (int i = 0; i < length; ++i) batch[used++] = line[i];
            batch[used] = '\0';
            ++count;
        } else flush();
        if (tx_time_get()-last_attitude >= TX_TIMER_TICKS_PER_SECOND/50) {
            flush();
            last_attitude=tx_time_get();
            const auto status=attitude::snapshot();
            const auto& s=status.estimate;
            core::Vec3 euler;
            core::attitude_euler(s.q_nb,euler);
            const auto micro=[](float v) { return std::lround(v*1000000.0F); };
            const auto degrees=[](float v) { return std::lround(v*180000.0F/core::kPi); };
            char line[512];
            std::snprintf(line,sizeof(line),
                "# ATT seq=%lu t=%lu%06lu q_u=%ld/%ld/%ld/%ld rpy_md=%ld/%ld/%ld total_yaw_md=%ld total_epoch=%lu bias_u=%ld/%ld/%ld aw_m=%ld dt_us=%ld run_us=%lu lat_us=%lu valid=%u yaw_abs=%u err=%u\r\n",
                static_cast<unsigned long>(s.metadata.sequence),
                static_cast<unsigned long>(s.metadata.measured_us/1000000),static_cast<unsigned long>(s.metadata.measured_us%1000000),
                micro(s.q_nb.w),micro(s.q_nb.x),micro(s.q_nb.y),micro(s.q_nb.z),
                degrees(euler.x),degrees(euler.y),degrees(euler.z),
                degrees(s.total_yaw_rad),
                static_cast<unsigned long>(s.total_epoch),
                micro(s.gyro_bias_rad_s.x),micro(s.gyro_bias_rad_s.y),micro(s.gyro_bias_rad_s.z),
                std::lround(status.accel_weight*1000),micro(status.dt_s),
                static_cast<unsigned long>(status.runtime_us),static_cast<unsigned long>(status.latency_us),
                s.metadata.valid,s.absolute_yaw_valid,static_cast<unsigned>(status.error));
            platform::write(line);
            const auto& r=status.rate;
            std::snprintf(line,sizeof(line),
                "# RATE seq=%lu rate_u=%ld/%ld/%ld accel_m=%ld/%ld/%ld valid=%u d_valid=%u err=%u\r\n",
                static_cast<unsigned long>(r.metadata.sequence),
                micro(r.body_rate_rad_s.x),micro(r.body_rate_rad_s.y),micro(r.body_rate_rad_s.z),
                std::lround(r.angular_accel_rad_s2.x*1000),std::lround(r.angular_accel_rad_s2.y*1000),
                std::lround(r.angular_accel_rad_s2.z*1000),
                r.metadata.valid,r.derivative_valid,static_cast<unsigned>(status.rate_error));
            platform::write(line);
        }
        if (tx_time_get() - last_status >= TX_TIMER_TICKS_PER_SECOND) {
            flush();
            last_status = tx_time_get();
            const auto status = imu::snapshot();
            char line[384];
            std::snprintf(line, sizeof(line),
                "# BMI088 init=%u err=%u aid=%02x gid=%02x src=%u reg=%02x expect=%02x got=%02x cpu=%lu tick=%lu\r\n"
                "# STATS a=%lu g=%lu missed=%lu/%lu spierr=%lu/%lu overlap=%lu/%lu stale=%lu/%lu latmax=%lu/%lu readmax=%lu/%lu logdrop=%lu wait=%lu valid=%u/%u\r\n",
                status.initialized, static_cast<unsigned>(status.info.error), status.info.accel_id,
                status.info.gyro_id, static_cast<unsigned>(status.info.sensor), status.info.reg,
                status.info.expected, status.info.observed, static_cast<unsigned long>(platform::cpu_hz()), last_status,
                static_cast<unsigned long>(status.stats[0].events), static_cast<unsigned long>(status.stats[1].events),
                static_cast<unsigned long>(status.stats[0].missed), static_cast<unsigned long>(status.stats[1].missed),
                static_cast<unsigned long>(status.stats[0].errors), static_cast<unsigned long>(status.stats[1].errors),
                static_cast<unsigned long>(status.stats[0].overlaps), static_cast<unsigned long>(status.stats[1].overlaps),
                static_cast<unsigned long>(status.stats[0].stale), static_cast<unsigned long>(status.stats[1].stale),
                static_cast<unsigned long>(status.stats[0].maximum_latency_us), static_cast<unsigned long>(status.stats[1].maximum_latency_us),
                static_cast<unsigned long>(status.stats[0].maximum_read_us), static_cast<unsigned long>(status.stats[1].maximum_read_us),
                static_cast<unsigned long>(status.dropped_logs), static_cast<unsigned long>(status.wait_timeouts),
                status.sample.accel.metadata.valid, status.sample.gyro.metadata.valid);
            platform::write(line);
            const auto micro = [](float value) { return std::lround(value*1000000.0F); };
            const auto& c = status.calibration;
            std::snprintf(line,sizeof(line),
                "# CAL gyro=%u accel=%u quality=%u err=%u a=%lu g=%lu restart=%lu bias_u=%ld/%ld/%ld accel_bias_u=%ld/%ld/%ld scale_u=%ld/%ld/%ld var_n=%ld temp_mc=%ld temp_valid=%u\r\n",
                status.gyro_calibrated,status.accel_calibrated,static_cast<unsigned>(c.quality),
                static_cast<unsigned>(status.calibration_error),
                static_cast<unsigned long>(status.calibration_samples[0]),static_cast<unsigned long>(status.calibration_samples[1]),
                static_cast<unsigned long>(status.calibration_restarts),
                micro(c.gyro_bias_rad_s.x),micro(c.gyro_bias_rad_s.y),micro(c.gyro_bias_rad_s.z),
                micro(c.accel_bias_m_s2.x),micro(c.accel_bias_m_s2.y),micro(c.accel_bias_m_s2.z),
                micro(c.accel_scale.x),micro(c.accel_scale.y),micro(c.accel_scale.z),
                std::lround(c.stationary_gyro_variance*1000000000.0F),std::lround(c.temperature_c*1000.0F),c.temperature_valid);
            platform::write(line);
            const auto& temp = status.temperature;
            const bool delta_valid = temp.valid && c.temperature_valid && status.gyro_calibrated;
            std::snprintf(line,sizeof(line),
                "# TEMP current_mc=%ld valid=%u delta_mc=%ld delta_valid=%u errors=%lu\r\n"
                "# PIPE pubmax=%lu/%lu procmax=%lu/%lu late=%lu/%lu\r\n",
                std::lround(temp.value_c*1000.0F),temp.valid,
                delta_valid ? std::lround((temp.value_c-c.temperature_c)*1000.0F) : 0L,delta_valid,
                static_cast<unsigned long>(temp.errors),
                static_cast<unsigned long>(status.stats[0].maximum_publication_latency_us),
                static_cast<unsigned long>(status.stats[1].maximum_publication_latency_us),
                static_cast<unsigned long>(status.stats[0].maximum_processing_us),
                static_cast<unsigned long>(status.stats[1].maximum_processing_us),
                static_cast<unsigned long>(status.stats[0].publication_late),
                static_cast<unsigned long>(status.stats[1].publication_late));
            platform::write(line);
            const auto attitude_status=attitude::snapshot();
            std::snprintf(line,sizeof(line),
                "# AHRS updates=%lu reject=%lu timing=%lu missed=%lu timeout=%lu late=%lu maxrun=%lu maxlat=%lu\r\n",
                static_cast<unsigned long>(attitude_status.updates),static_cast<unsigned long>(attitude_status.rejections),
                static_cast<unsigned long>(attitude_status.timing_errors),static_cast<unsigned long>(attitude_status.missed),
                static_cast<unsigned long>(attitude_status.timeouts),static_cast<unsigned long>(attitude_status.late),
                static_cast<unsigned long>(attitude_status.maximum_runtime_us),static_cast<unsigned long>(attitude_status.maximum_latency_us));
            platform::write(line);
            const auto& a = status.sample.accel;
            const auto& g = status.sample.gyro;
            std::snprintf(line,sizeof(line),
                "# CORR a_seq=%lu ax_u=%ld ay_u=%ld az_u=%ld g_seq=%lu gx_u=%ld gy_u=%ld gz_u=%ld valid=%u/%u\r\n",
                static_cast<unsigned long>(a.metadata.sequence),micro(a.value.x),micro(a.value.y),micro(a.value.z),
                static_cast<unsigned long>(g.metadata.sequence),micro(g.value.x),micro(g.value.y),micro(g.value.z),
                a.metadata.valid,g.metadata.valid);
            platform::write(line);
            const auto flight_status=flight::snapshot();
            std::snprintf(line,sizeof(line),
                "# FLIGHT state=%u allowed=%u block=%lu fault=%lu reset=%lu rc=%lu dry=1 batches=%lu stopped=%lu frames=%04x/%04x/%04x/%04x stop_reason=%lu\r\n",
                static_cast<unsigned>(flight_status.status.state),flight_status.status.output_allowed,
                static_cast<unsigned long>(flight_status.status.arm_block_reasons),
                static_cast<unsigned long>(flight_status.status.fault_reasons),
                static_cast<unsigned long>(flight_status.control_reset_sequence),
                static_cast<unsigned long>(flight_status.rc_updates),static_cast<unsigned long>(flight_status.batches),
                static_cast<unsigned long>(flight_status.stop_batches),
                static_cast<unsigned>(flight_status.software_output.frames[0]),
                static_cast<unsigned>(flight_status.software_output.frames[1]),
                static_cast<unsigned>(flight_status.software_output.frames[2]),
                static_cast<unsigned>(flight_status.software_output.frames[3]),
                static_cast<unsigned long>(flight_status.software_output.stop_reasons));
            platform::write(line);
            const auto& receiver=flight_status.receiver;
            const auto receiver_stats=rc::statistics();
            const auto now=platform::time_us();
            const auto age=now>=receiver.metadata.measured_us ? now-receiver.metadata.measured_us : 0;
            std::snprintf(line,sizeof(line),
                "# RC seq=%lu age_us=%lu fresh=%u lost=%u failsafe=%u map=%u bytes=%lu err=%lu drop=%lu bad=%lu reset=%lu ch=",
                static_cast<unsigned long>(receiver.metadata.sequence),
                static_cast<unsigned long>(age>0xffffffffU ? 0xffffffffU : age),
                receiver.metadata.valid && core::is_fresh(receiver.metadata.measured_us,now,100000),
                receiver.frame_lost,receiver.failsafe,flight_status.receiver_mapping_ready,
                static_cast<unsigned long>(receiver_stats.bytes),static_cast<unsigned long>(receiver_stats.errors),
                static_cast<unsigned long>(receiver_stats.drops),
                static_cast<unsigned long>(flight_status.receiver_stats.malformed),
                static_cast<unsigned long>(receiver_stats.restarts));
            unsigned length=static_cast<unsigned>(std::strlen(line));
            for (unsigned i=0;i<16;++i) length+=static_cast<unsigned>(std::snprintf(line+length,sizeof(line)-length,
                "%u%c",static_cast<unsigned>(receiver.channels[i]),i==15 ? '\r' : '/'));
            line[length++]='\n'; line[length]='\0';
            platform::write(line);
            const auto& pilot=flight_status.pilot;
            std::snprintf(line,sizeof(line),
                "# PILOT seq=%lu fresh=%u rpy_m=%ld/%ld/%ld throttle_m=%ld arm=%u mode=%u estop=%u failsafe=%u\r\n",
                static_cast<unsigned long>(pilot.metadata.sequence),
                core::is_usable(pilot,now,100000),
                std::lround(pilot.roll*1000),std::lround(pilot.pitch*1000),std::lround(pilot.yaw*1000),
                std::lround(pilot.collective*1000),pilot.arm_request,static_cast<unsigned>(pilot.mode),
                pilot.emergency_stop,pilot.receiver_failsafe);
            platform::write(line);
        }
    }
}
}

unsigned int app_start(void) {
    platform::start_timer();
    const auto attitude_status = self_flight::attitude::start();
    if (attitude_status != TX_SUCCESS) return attitude_status;
    const auto status = self_flight::imu::start();
    if (status != TX_SUCCESS) return status;
    const auto receiver_status=self_flight::rc::start();
    if (receiver_status!=TX_SUCCESS) return receiver_status;
    return tx_thread_create(&log_thread, const_cast<char*>("usb_log"), run, 0,
                            stack, sizeof(stack), 15, 15, TX_NO_TIME_SLICE, TX_AUTO_START);
}
