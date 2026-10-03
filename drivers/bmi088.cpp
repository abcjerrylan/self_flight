#include "bmi088.hpp"
#include <cstring>

namespace self_flight::bmi088 {
namespace {
std::int16_t signed16(const std::uint8_t* p) {
    const int value = p[0] | (static_cast<int>(p[1]) << 8);
    return static_cast<std::int16_t>(value >= 32768 ? value - 65536 : value);
}
struct Setting { Sensor sensor; std::uint8_t reg, value, mask; };
constexpr Setting settings[] = {
    {Sensor::Accel, 0x7d, 0x04, 0xff}, // Power on.
    {Sensor::Accel, 0x7c, 0x00, 0xff}, // Active mode.
    {Sensor::Accel, 0x40, 0xab, 0xff}, // Normal filter, 800Hz.
    {Sensor::Accel, 0x41, 0x02, 0x03}, // +/-12g.
    {Sensor::Accel, 0x53, 0x0a, 0x1e}, // INT1 push-pull, active high.
    {Sensor::Accel, 0x54, 0x0a, 0x1e}, // INT2 push-pull, active high.
    {Sensor::Accel, 0x58, 0x44, 0x77}, // DRDY to both pins; only one is wired.
    {Sensor::Gyro, 0x11, 0x00, 0xff}, // Normal mode.
    {Sensor::Gyro, 0x0f, 0x00, 0x07}, // +/-2000 deg/s.
    {Sensor::Gyro, 0x10, 0x02, 0x07}, // 1000Hz, 116Hz bandwidth; bit7 is read-only.
    {Sensor::Gyro, 0x16, 0x05, 0x0f}, // INT3/4 push-pull, active high.
    {Sensor::Gyro, 0x18, 0x81, 0x85}, // DRDY to both pins.
    {Sensor::Gyro, 0x15, 0x80, 0xc0}, // Enable DRDY last.
};
}

Raw decode(const std::uint8_t* data, Sensor sensor) {
    Raw raw{signed16(data), signed16(data + 2), signed16(data + 4), 0};
    if (sensor == Sensor::Accel)
        raw.sensor_time = data[6] | (std::uint32_t{data[7]} << 8) | (std::uint32_t{data[8]} << 16);
    return raw;
}

core::Vec3 to_si(const Raw& raw, Sensor sensor) {
    const float scale = sensor == Sensor::Accel ? 12.0F * 9.80665F / 32768.0F
                                                : 2000.0F * core::kPi / (180.0F * 32768.0F);
    return {raw.x * scale, raw.y * scale, raw.z * scale};
}

bool Driver::read_registers(Sensor sensor, std::uint8_t reg, std::uint8_t* out, std::size_t count) {
    const std::size_t offset = sensor == Sensor::Accel ? 2 : 1;
    std::uint8_t tx[11]{}, rx[11]{};
    if (count > sizeof(tx) - offset) return false;
    tx[0] = reg | 0x80;
    if (!bus_.transfer(bus_.context, sensor, tx, rx, count + offset)) return false;
    std::memcpy(out, rx + offset, count);
    return true;
}

bool Driver::write_register(Sensor sensor, std::uint8_t reg, std::uint8_t value) {
    const std::uint8_t tx[]{reg, value};
    std::uint8_t rx[2]{};
    const bool ok = bus_.transfer(bus_.context, sensor, tx, rx, sizeof(tx));
    bus_.delay_ms(bus_.context, 2); // Also meets the 1ms suspend-mode write spacing.
    return ok;
}

Info Driver::initialize() {
    Info info{};
    std::uint8_t dummy{};
    bus_.delay_ms(bus_.context, 40);
    if (!read_registers(Sensor::Accel, 0x00, &dummy, 1) ||
        !write_register(Sensor::Accel, 0x7e, 0xb6) ||
        !write_register(Sensor::Gyro, 0x14, 0xb6)) {
        info.error = Error::Transfer;
        return info;
    }
    bus_.delay_ms(bus_.context, 40);
    // Discard the first accel access after reset, then obtain real chip IDs.
    if (!read_registers(Sensor::Accel, 0x00, &dummy, 1) ||
        !read_registers(Sensor::Accel, 0x00, &info.accel_id, 1) ||
        !read_registers(Sensor::Gyro, 0x00, &info.gyro_id, 1)) {
        info.error = Error::Transfer;
        return info;
    }
    if (info.accel_id != 0x1e || info.gyro_id != 0x0f) {
        info.error = Error::ChipId;
        return info;
    }
    for (const auto& setting : settings) {
        info.sensor = setting.sensor;
        info.reg = setting.reg;
        info.expected = setting.value;
        if (!write_register(setting.sensor, setting.reg, setting.value) ||
            !read_registers(setting.sensor, setting.reg, &info.observed, 1)) {
            info.error = Error::Transfer;
            return info;
        }
        if ((info.observed & setting.mask) != (setting.value & setting.mask)) {
            info.error = Error::Configuration;
            return info;
        }
    }
    info.sensor = Sensor::Accel;
    info.expected = 0;
    if (!read_registers(Sensor::Accel, 0x02, &info.observed, 1)) info.error = Error::Transfer;
    else if (info.observed != 0) info.error = Error::SensorFault;
    info.reg = 0x02;
    return info;
}

bool Driver::read(Sensor sensor, Raw& raw) {
    std::uint8_t data[9]{};
    if (!read_registers(sensor, sensor == Sensor::Accel ? 0x12 : 0x02,
                        data, sensor == Sensor::Accel ? 9 : 6)) return false;
    raw = decode(data, sensor);
    return true;
}

bool Driver::read_temperature(float& temperature_c) {
    std::uint8_t data[2]{};
    if (!read_registers(Sensor::Accel, 0x22, data, 2)) return false;
    const int raw = (static_cast<int>(data[0]) << 3) | (data[1] >> 5);
    if (raw == 1024) return false; // Datasheet invalid temperature code.
    temperature_c = 23.0F + 0.125F * (raw >= 1024 ? raw-2048 : raw);
    return true;
}
}
