#include "self_flight/core/imu.hpp"
#include <iostream>
#include <string>

using namespace self_flight::core;
namespace {
int failures{};
#define CHECK(x) do { if (!(x)) { std::cerr << __LINE__ << ": " << #x << '\n'; ++failures; } } while(false)
VectorSample sample(std::uint32_t sequence, TimestampUs time, bool valid=true) {
    return {{time,time+30,sequence,valid},{0,0,1}};
}
void consumption() {
    ImuCursor cursor;
    ImuSample latest{sample(0,1000),sample(0,900)},out;
    CHECK(cursor.take(latest,1030,out) && out.accel_is_new);
    CHECK(!cursor.take(latest,1030,out));
    latest.gyro=sample(1,2000);
    CHECK(cursor.take(latest,2030,out) && !out.accel_is_new && out.accel.metadata.valid);
    // The sampler flag is irrelevant; a subsequent accel update must be observed.
    latest.accel=sample(1,2100); latest.accel_is_new=false;
    CHECK(!cursor.take(latest,2130,out));
    latest.gyro=sample(2,3000);
    CHECK(cursor.take(latest,3030,out) && out.accel_is_new);
    ImuCursor independent;
    CHECK(independent.take(latest,3030,out) && out.accel_is_new);
}
void timing() {
    ImuCursor cursor;
    ImuSample latest{sample(1,1000),sample(1,1100)},out;
    CHECK(cursor.take(latest,1130,out) && !out.accel.metadata.valid && !out.accel_is_new);
    latest.gyro=sample(2,2000);
    CHECK(cursor.take(latest,2030,out) && out.accel_is_new);
    latest.gyro=sample(3,3000); latest.accel=sample(2,2900,false);
    CHECK(cursor.take(latest,3030,out) && !out.accel_is_new);
    latest.gyro=sample(4,4000); latest.accel.metadata.valid=true;
    CHECK(cursor.take(latest,4030,out) && out.accel_is_new);
    latest.gyro=sample(5,5000);
    const auto saved=out.gyro.metadata.sequence;
    CHECK(!cursor.take(latest,11000,out) && out.gyro.metadata.sequence==saved);
    CHECK(!cursor.take(latest,5010,out)); // Not available yet.
    latest.gyro=sample(5,3900);
    CHECK(!cursor.take(latest,4030,out)); // Backward measurement time.
}
void wrap() {
    ImuCursor cursor;
    ImuSample latest{sample(0xffffffffU,1000),sample(0xffffffffU,900)},out;
    CHECK(cursor.take(latest,1030,out) && out.accel_is_new);
    latest={sample(0,2000),sample(0,1900)};
    CHECK(cursor.take(latest,2030,out) && out.accel_is_new);
    latest.gyro=sample(0xffffffffU,3000);
    CHECK(!cursor.take(latest,3030,out)); // Backward sequence, not wrap.
    latest.gyro=sample(2,4000); // A skipped gyro sample can be reported to the solver.
    latest.accel=sample(0xffffffffU,3900);
    CHECK(cursor.take(latest,4030,out) && !out.accel_is_new);
}
}
int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const std::string test=argv[1];
    if (test=="consumption") consumption();
    else if (test=="timing") timing();
    else if (test=="wrap") wrap();
    else return 2;
    return failures ? 1 : 0;
}
