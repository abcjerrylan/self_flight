#pragma once
#include "sbus.hpp"

namespace self_flight::board {
// Measured AETR. Pulling the right stick back commands positive FRD pitch (nose up).
// Arm awaits a measured switch; TRS CH12 (index 11) carries RSSI, not a control.
inline constexpr sbus::ChannelMap receiver_channels{
    0,1,3,2,sbus::kUnassigned,sbus::kUnassigned,sbus::kUnassigned,
    173,992,1811,{false,true,false}};
}
