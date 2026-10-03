#include "bmi088.hpp"
#include "profile.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

using namespace self_flight;
namespace {
int failures = 0;
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while (false)
struct FakeBus {
    std::array<std::array<std::uint8_t, 128>, 2> registers{};
    bool fail = false, bad_config = false;
    unsigned waits = 0, reads = 0;
    FakeBus() { registers[0][0] = 0x1e; registers[1][0] = 0x0f; }
    static bool transfer(void* context, bmi088::Sensor sensor, const std::uint8_t* tx,
                         std::uint8_t* rx, std::size_t size) {
        auto& bus = *static_cast<FakeBus*>(context);
        if (bus.fail) return false;
        const unsigned index = static_cast<unsigned>(sensor);
        const auto reg = tx[0] & 0x7f;
        if (tx[0] & 0x80) {
            ++bus.reads;
            const unsigned offset = index == 0 ? 2 : 1;
            CHECK(size > offset);
            std::memset(rx, 0xcc, size); // Dummy bytes must not appear in decoded data.
            for (std::size_t i = offset; i < size; ++i) rx[i] = bus.registers[index][reg + i - offset];
            if (index == 1 && reg == 0x10) rx[offset] |= 0x80; // Read-only bit7.
        } else {
            CHECK(size == 2);
            if (reg != 0x7e && reg != 0x14 && !(bus.bad_config && reg == 0x41))
                bus.registers[index][reg] = tx[1];
        }
        return true;
    }
    static void delay(void* context, unsigned ms) { static_cast<FakeBus*>(context)->waits += ms; }
    bmi088::Bus bus() { return {this, transfer, delay}; }
};
void init() {
    FakeBus bus;
    bmi088::Driver driver(bus.bus());
    const auto info = driver.initialize();
    CHECK(info.error == bmi088::Error::None);
    CHECK(info.accel_id == 0x1e && info.gyro_id == 0x0f);
    CHECK(bus.registers[0][0x40] == 0xab && bus.registers[0][0x41] == 2);
    CHECK(bus.registers[0][0x58] == 0x44 && bus.registers[1][0x18] == 0x81);
    CHECK(bus.registers[1][0x10] == 2 && bus.registers[1][0x15] == 0x80);
    CHECK(bus.waits >= 100);
}
void errors() {
    FakeBus bad_id;
    bad_id.registers[1][0] = 0;
    CHECK(bmi088::Driver(bad_id.bus()).initialize().error == bmi088::Error::ChipId);
    FakeBus bad_io;
    bad_io.fail = true;
    bmi088::Driver driver(bad_io.bus());
    CHECK(driver.initialize().error == bmi088::Error::Transfer);
    bmi088::Raw raw{1,2,3,4};
    CHECK(!driver.read(bmi088::Sensor::Accel, raw));
    CHECK(raw.x == 1 && raw.sensor_time == 4);
    FakeBus bad_config;
    bad_config.bad_config = true;
    const auto info = bmi088::Driver(bad_config.bus()).initialize();
    CHECK(info.error == bmi088::Error::Configuration && info.reg == 0x41);
    FakeBus fault;
    fault.registers[0][2] = 1;
    CHECK(bmi088::Driver(fault.bus()).initialize().error == bmi088::Error::SensorFault);
}
void data() {
    const std::uint8_t bytes[]{0xff,0x7f,0x00,0x80,0xff,0xff,0x56,0x34,0x12};
    const auto raw = bmi088::decode(bytes, bmi088::Sensor::Accel);
    CHECK(raw.x == 32767 && raw.y == -32768 && raw.z == -1 && raw.sensor_time == 0x123456);
    const auto gyro = bmi088::decode(bytes, bmi088::Sensor::Gyro);
    CHECK(gyro.sensor_time == 0);
    const auto acc_si = bmi088::to_si({0,0,-32768,0}, bmi088::Sensor::Accel);
    CHECK(std::fabs(acc_si.z + 12 * 9.80665F) < 1e-4F);
    const auto gyro_si = bmi088::to_si({0,-16384,0,0}, bmi088::Sensor::Gyro);
    CHECK(std::fabs(gyro_si.y + 1000 * core::kPi / 180) < 1e-4F);
    FakeBus temperature;
    temperature.registers[0][0x22]=0; temperature.registers[0][0x23]=0x60;
    bmi088::Driver thermal(temperature.bus()); float c=0;
    CHECK(thermal.read_temperature(c) && std::fabs(c-23.375F)<1e-6F);
    temperature.registers[0][0x22]=0xc1; temperature.registers[0][0x23]=0;
    CHECK(thermal.read_temperature(c) && c==-40);
    temperature.registers[0][0x22]=0x80;
    CHECK(!thermal.read_temperature(c) && c==-40);
    temperature.fail=true;
    CHECK(!thermal.read_temperature(c) && c==-40);
    FakeBus bus;
    std::memcpy(bus.registers[0].data() + 0x12, bytes, sizeof(bytes));
    std::memcpy(bus.registers[1].data() + 0x02, bytes, 6);
    bmi088::Driver driver(bus.bus());
    bmi088::Raw sample{};
    CHECK(driver.read(bmi088::Sensor::Accel, sample));
    CHECK(sample.x == 32767 && sample.sensor_time == 0x123456);
    CHECK(driver.read(bmi088::Sensor::Gyro, sample));
    CHECK(sample.y == -32768 && sample.sensor_time == 0);
}
void mounting() {
    const auto transformed = board::sensor_to_body({1,2,3});
    CHECK(transformed.x == -2 && transformed.y == -1 && transformed.z == -3);
    CHECK(core::dot(transformed, transformed) == 14);
    const auto x = board::sensor_to_body({1,0,0});
    const auto y = board::sensor_to_body({0,1,0});
    const auto z = board::sensor_to_body({0,0,1});
    const auto cross = core::cross(x,y);
    CHECK(cross.x == z.x && cross.y == z.y && cross.z == z.z);
}
}
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const std::string test = argv[1];
    if (test == "init") init();
    else if (test == "errors") errors();
    else if (test == "data") data();
    else if (test == "mounting") mounting();
    else return 2;
    return failures ? 1 : 0;
}
