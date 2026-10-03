#include "self_flight/core/calibration.hpp"
#include <iomanip>
#include <iostream>
#include <string>

using namespace self_flight::core;
void vector(Vec3 v) { std::cout << '[' << v.x << ',' << v.y << ',' << v.z << ']'; }
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    std::cout << std::setprecision(9);
    if (std::string(argv[1]) == "fit") {
        std::array<Vec3,6> means{};
        for (auto& v : means) if (!(std::cin >> v.x >> v.y >> v.z)) return 2;
        Calibration result;
        const auto error = fit_accelerometer(means,result);
        if (error != CalibrationError::None) {
            std::cerr << "Six-face fit rejected, error=" << static_cast<unsigned>(error) << '\n';
            return 1;
        }
        std::cout << "{\"accel_bias_m_s2\":"; vector(result.accel_bias_m_s2);
        std::cout << ",\"accel_scale\":"; vector(result.accel_scale);
        std::cout << "}\n";
        return 0;
    }
    if (std::string(argv[1]) != "window") return 2;
    StationaryCalibration window;
    char source;
    VectorSample sample;
    unsigned valid;
    while (std::cin >> source >> sample.metadata.sequence >> sample.metadata.measured_us >>
           sample.metadata.available_us >> sample.value.x >> sample.value.y >> sample.value.z >> valid) {
        if ((source != 'A' && source != 'G') || valid > 1) return 2;
        sample.metadata.valid = valid;
        window.add(source == 'G',sample);
        GyroEstimate result;
        if (!window.estimate(result)) continue;
        std::cout << "{\"gyro_bias_rad_s\":"; vector(result.bias);
        std::cout << ",\"gyro_variance\":"; vector(result.variance);
        std::cout << ",\"accel_mean_m_s2\":"; vector(result.accel_mean);
        std::cout << ",\"measured_us\":" << result.measured_us << "}\n";
        return 0;
    }
    std::cerr << "No continuous qualified 3-second stationary window\n";
    return 1;
}
