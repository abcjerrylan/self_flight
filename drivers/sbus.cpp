#include "sbus.hpp"
#include <algorithm>

namespace self_flight::sbus {
void Parser::reset() {
    stats_.partial_resets += used_ != 0;
    used_=0; seen_=false;
}
bool Parser::push(std::uint8_t byte, core::TimestampUs now, Frame& out) {
    if (seen_ && now < last_time_) { used_=0; ++stats_.time_errors; return false; }
    // 3ms also detects the gap between TRS 150Hz frames (25 bytes take 3ms).
    if (seen_ && now-last_time_ > 3000 && used_) { used_=0; ++stats_.partial_resets; }
    last_time_=now; seen_=true;
    if (used_ == 0 && byte != 0x0f) return false;
    bytes_[used_++]=byte;
    if (used_ != bytes_.size()) return false;
    const auto footer=bytes_[24];
    if ((footer != 0 && (footer & 0xcf) != 4) || (bytes_[23] & 0xf0)) {
        ++stats_.malformed;
        // Preserve a possible later header after a dropped byte, without unbounded storage.
        unsigned start=1;
        while (start<bytes_.size() && bytes_[start]!=0x0f) ++start;
        used_=static_cast<unsigned>(bytes_.size())-start;
        for (unsigned n=0;n<used_;++n) bytes_[n]=bytes_[n+start];
        return false;
    }
    Frame decoded;
    decoded.metadata={now,now,stats_.frames+1,true};
    for (unsigned channel=0;channel<16;++channel) {
        const auto bit=channel*11, index=1+bit/8, shift=bit%8;
        const std::uint32_t packed=bytes_[index] | (static_cast<std::uint32_t>(bytes_[index+1])<<8) |
                                   (static_cast<std::uint32_t>(bytes_[index+2])<<16);
        decoded.channels[channel]=static_cast<std::uint16_t>((packed>>shift) & 2047);
    }
    const auto flags=bytes_[23];
    decoded.digital17=flags&1; decoded.digital18=flags&2;
    decoded.frame_lost=flags&4; decoded.failsafe=flags&8;
    ++stats_.frames; stats_.lost += decoded.frame_lost; stats_.failsafe += decoded.failsafe;
    used_=0; out=decoded;
    return true;
}
bool valid_mapping(const ChannelMap& m) {
    if (!(m.low < m.center && m.center < m.high && m.high <= 2047)) return false;
    const std::array<std::uint8_t,7> channels{m.roll,m.pitch,m.yaw,m.throttle,m.arm,m.attitude_mode,m.emergency_stop};
    for (unsigned i=0;i<channels.size();++i) {
        if (i>=5 && channels[i]==kUnassigned) continue;
        if (channels[i]>=16) return false;
        for (unsigned j=0;j<i;++j) if (channels[j]==channels[i]) return false;
    }
    return true;
}
bool map_pilot(const Frame& frame, const ChannelMap& m, core::PilotCommand& out) {
    const auto& meta=frame.metadata;
    if (!valid_mapping(m) || !meta.valid || meta.measured_us > meta.available_us ||
        (frame.frame_lost && !frame.failsafe)) return false;
    for (auto value : frame.channels) if (value>2047) return false;
    const auto axis=[&](unsigned channel, unsigned index) {
        const float v=frame.channels[channel];
        const float scaled=v <= m.center ? (v-m.center)/(m.center-m.low) : (v-m.center)/(m.high-m.center);
        return std::clamp(scaled,-1.0F,1.0F)*(m.reversed[index] ? -1.0F : 1.0F);
    };
    const auto on=[&](unsigned channel) {
        return channel<16 && frame.channels[channel]>m.center+(m.high-m.center)/3;
    };
    core::PilotCommand command;
    command.metadata=meta;
    command.roll=axis(m.roll,0); command.pitch=axis(m.pitch,1); command.yaw=axis(m.yaw,2);
    command.collective=std::clamp((static_cast<float>(frame.channels[m.throttle])-m.low)/(m.high-m.low),0.0F,1.0F);
    command.arm_request=on(m.arm);
    command.mode=on(m.attitude_mode) ? core::FlightMode::Attitude : core::FlightMode::Rate;
    command.emergency_stop=on(m.emergency_stop); command.receiver_failsafe=frame.failsafe;
    out=command;
    return true;
}
}
