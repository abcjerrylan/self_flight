"""Validate six stationary captures and fit using the same C++ core as firmware."""
import argparse
import importlib.util
import json
import math
import statistics
import subprocess
from pathlib import Path

spec = importlib.util.spec_from_file_location("replay", Path(__file__).with_name("replay-imu.py"))
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
BOARD_ID, SENSOR_ID = 0x74323535, 0xB0880F1E
FACES = ("x-plus", "x-minus", "y-plus", "y-minus", "z-plus", "z-minus")


def load_stationary(path):
    with path.open(encoding="utf-8-sig") as stream:
        rows = list(replay.records(stream))
    summary = replay.summarize(rows)
    for source in ("A", "G"):
        data = [r for r in rows if r["sensor"] == source]
        stats = summary[source]
        if (len(data) < (2000 if source == "A" else 2500) or stats["invalid"]
                or stats["sequence_gaps"] or stats["duplicates"] or stats["backward"]
                or data[-1]["measured_us"]-data[0]["measured_us"] < 3_000_000
                or stats["dt_us_max"] > (4000 if source == "A" else 3000)):
            raise ValueError(f"{path}: insufficient/invalid/discontinuous {source} samples")
        stddev = stats["body_stddev_si"]
        if max(stddev) > (0.2 if source == "A" else 0.03):
            raise ValueError(f"{path}: moving/noisy {source} window")
        if source == "G" and math.sqrt(sum(v*v for v in stats["body_mean_si"])) > 0.05:
            raise ValueError(f"{path}: mean angular rate too high")
        if source == "A" and not 8.8 <= stats["mean_force_norm_m_s2"] <= 10.8:
            raise ValueError(f"{path}: force magnitude outside stationary range")
    return rows, summary


def native(executable, mode, text):
    result = subprocess.run([str(executable), mode], input=text, capture_output=True, text=True)
    if result.returncode:
        raise ValueError(result.stderr.strip() or f"native calibration {mode} failed")
    return json.loads(result.stdout)


def fit(directory, executable):
    means, report, all_rows = {}, {}, {}
    for face in FACES:
        rows, report[face] = load_stationary(directory / f"body-{face}.csv")
        all_rows[face] = rows
        accel = [r for r in rows if r["sensor"] == "A"]
        means[face] = [statistics.mean(r["sensor_si"][i] for r in accel) for i in range(3)]
    # Body=(-sensor_y,-sensor_x,-sensor_z); reorder physical body faces to sensor faces.
    sensor_order = ("y-minus", "y-plus", "x-minus", "x-plus", "z-minus", "z-plus")
    calibration = native(executable, "fit", "\n".join(" ".join(map(str, means[f])) for f in sensor_order))
    reference = all_rows["z-minus"]
    window = "\n".join(f'{r["sensor"]} {r["sequence"]} {r["measured_us"]} {r["available_us"]} '
                       + " ".join(map(str, r["sensor_si"])) + f' {r["valid"]}' for r in reference)
    gyro = native(executable, "window", window)
    calibration.update(schema_version=1, board_identity=BOARD_ID, sensor_identity=SENSOR_ID,
                       axes="sensor", gyro_bias_rad_s=gyro["gyro_bias_rad_s"],
                       stationary_gyro_variance=max(gyro["gyro_variance"]), measured_us=gyro["measured_us"],
                       quality="Accepted", temperature_valid=False)
    for face, rows in all_rows.items():
        corrected = [[(r["sensor_si"][i]-calibration["accel_bias_m_s2"][i])*calibration["accel_scale"][i]
                      for i in range(3)] for r in rows if r["sensor"] == "A"]
        report[face]["corrected_force_mean_m_s2"] = statistics.mean(math.sqrt(sum(v*v for v in xyz)) for xyz in corrected)
    return calibration, dict(faces=report, gyro_reference_window=gyro)


def header(calibration):
    def vector(values):
        return "{" + ", ".join(f"{v:.9f}F" for v in values) + "}"
    return f'''#pragma once
#include "self_flight/core/data.hpp"

namespace self_flight::board {{
constexpr std::uint32_t kCalibrationBoard = 0x74323535;
constexpr std::uint32_t kCalibrationSensor = 0xb0880f1e;
// This board's accepted six-face fit. Gyro bias is re-estimated each boot.
inline core::Calibration accelerometer_calibration() {{
    core::Calibration c;
    c.board_identity = kCalibrationBoard;
    c.sensor_identity = kCalibrationSensor;
    c.accel_bias_m_s2 = {vector(calibration['accel_bias_m_s2'])};
    c.accel_scale = {vector(calibration['accel_scale'])};
    c.quality = core::CalibrationQuality::Accepted;
    return c;
}}
}}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--native", type=Path, default=Path("build/host-debug/tests/host/calibration_fit.exe"))
    parser.add_argument("--header", type=Path)
    args = parser.parse_args()
    try:
        calibration, report = fit(args.directory, args.native.resolve())
    except (ValueError, OSError) as error:
        parser.exit(1, f"Calibration rejected: {error}\n")
    (args.directory/"calibration.json").write_text(json.dumps(calibration,indent=2)+"\n",encoding="utf-8")
    (args.directory/"calibration-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    if args.header:
        args.header.write_text(header(calibration),encoding="utf-8")
    print(json.dumps(calibration,indent=2))


if __name__ == "__main__":
    main()
