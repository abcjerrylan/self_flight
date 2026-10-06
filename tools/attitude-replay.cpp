#include "self_flight/core/attitude.hpp"
#include "self_flight/core/imu.hpp"
#include <iomanip>
#include <iostream>

using namespace self_flight::core;
int main() {
    Mahony estimator;
    ImuCursor cursor;
    ImuSample latest,selected;
    VectorSample sample;
    char sensor; unsigned valid;
    std::cout<<std::setprecision(9)
        <<"sequence,measured_us,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,bx,by,bz,weight,valid,error,total_yaw_deg,total_epoch\n";
    while (std::cin>>sensor>>sample.metadata.sequence>>sample.metadata.measured_us>>
        sample.metadata.available_us>>sample.value.x>>sample.value.y>>sample.value.z>>valid) {
        if ((sensor!='A' && sensor!='G') || valid>1) return 2;
        sample.metadata.valid=valid;
        if (sensor=='A') { latest.accel=sample; continue; }
        latest.gyro=sample;
        if (!cursor.take(latest,sample.metadata.available_us,selected)) continue;
        const auto error=estimator.update(selected,sample.metadata.available_us);
        const auto& s=estimator.state();
        Vec3 euler; attitude_euler(s.q_nb,euler);
        std::cout<<sample.metadata.sequence<<','<<sample.metadata.measured_us<<','
            <<s.q_nb.w<<','<<s.q_nb.x<<','<<s.q_nb.y<<','<<s.q_nb.z<<','
            <<euler.x*180/kPi<<','<<euler.y*180/kPi<<','<<euler.z*180/kPi<<','
            <<s.gyro_bias_rad_s.x<<','<<s.gyro_bias_rad_s.y<<','<<s.gyro_bias_rad_s.z<<','
            <<estimator.accel_weight()<<','<<s.metadata.valid<<','<<static_cast<unsigned>(error)<<','
            <<s.total_yaw_rad*180/kPi<<','<<s.total_epoch<<'\n';
    }
    return std::cin.eof() ? 0 : 2;
}
