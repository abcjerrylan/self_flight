#pragma once
#include "self_flight/core/math.hpp"
#include <cstddef>
#include <cstdint>

namespace self_flight::bmi088 {
enum class Sensor : std::uint8_t { Accel, Gyro };
struct Bus {
    void* context;
    bool (*transfer)(void*, Sensor, const std::uint8_t*, std::uint8_t*, std::size_t);
    void (*delay_ms)(void*, unsigned);
};
enum class Error : std::uint8_t { None, Transfer, ChipId, Configuration, SensorFault };
struct Info {
    Error error{Error::None};
    Sensor sensor{Sensor::Accel};
    std::uint8_t accel_id{}, gyro_id{}, reg{}, expected{}, observed{};
};
struct Raw {
    std::int16_t x{}, y{}, z{};
    std::uint32_t sensor_time{}; // Accel's 24-bit counter; not an MCU timestamp.
};
Raw decode(const std::uint8_t* data, Sensor sensor);
core::Vec3 to_si(const Raw& raw, Sensor sensor); // Sensor axes, +/-12g and +/-2000 deg/s.

class Driver {
public:
    explicit Driver(Bus bus) : bus_(bus) {}
    Info initialize();
    bool read(Sensor sensor, Raw& raw);
    bool read_temperature(float& temperature_c);
private:
    bool read_registers(Sensor, std::uint8_t, std::uint8_t*, std::size_t);
    bool write_register(Sensor, std::uint8_t, std::uint8_t);
    Bus bus_;
};
}
