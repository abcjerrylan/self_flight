"""Replay calibrated IMU through the same C++ Mahony core as the firmware."""
import argparse
import csv
import importlib.util
import io
import json
import math
import statistics
import subprocess
from pathlib import Path

spec = importlib.util.spec_from_file_location("imu_replay", Path(__file__).with_name("replay-imu.py"))
imu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imu)


def calibration_from_log(lines):
    for line in lines:
        if not line.startswith("# CAL "):
            continue
        fields = dict(field.split("=", 1) for field in line.split() if "=" in field)
        if any(fields.get(key) != value for key, value in (("gyro", "1"), ("accel", "1"), ("quality", "2"))):
            continue
        vectors = {key: [int(v)/1e6 for v in fields[field].split("/")]
                   for key, field in (("gyro_bias_rad_s", "bias_u"), ("accel_bias_m_s2", "accel_bias_u"), ("accel_scale", "scale_u"))}
        return dict(schema_version=1, board_identity=0x74323535, sensor_identity=0xB0880F1E,
                    axes="sensor", quality="Accepted", **vectors)
    raise ValueError("no accepted # CAL; provide --calibration for older IMU logs")


def native_replay(rows, executable):
    text = "\n".join(f'{r["sensor"]} {r["sequence"]} {r["measured_us"]} {r["available_us"]} '
                     + " ".join(map(str, r["body_si"])) + f' {r["valid"]}' for r in rows)
    result = subprocess.run([str(executable)], input=text, text=True, capture_output=True)
    if result.returncode:
        raise ValueError(result.stderr.strip() or f"native replay failed ({result.returncode})")
    return result.stdout


def summarize(text):
    rows = list(csv.DictReader(io.StringIO(text)))
    usable = [row for row in rows if row["valid"] == "1"]
    if not usable:
        raise ValueError("no valid attitude frames; check calibration, gravity and timing")
    if any(row.get("total_kind") != "body" for row in usable):
        raise ValueError("native replay lacks total_kind=body; rebuild the matching C++ executable")
    norms = [math.sqrt(sum(float(row[k])**2 for k in ("qw", "qx", "qy", "qz"))) for row in usable]
    angles = [[float(row[key]) for row in usable] for key in ("roll_deg", "pitch_deg", "yaw_deg")]
    totals = [[float(row[key]) for row in usable] for key in ("total_roll_deg", "total_pitch_deg", "total_yaw_deg")]
    yaw_change = sum((after-before+180)%360-180 for before, after in zip(angles[2], angles[2][1:]))
    return dict(rows=len(rows), valid=len(usable), invalid=len(rows)-len(usable),
                error_counts={error: sum(row["error"] == error for row in rows) for error in sorted({r["error"] for r in rows})},
                quaternion_norm_error_max=max(abs(n-1) for n in norms),
                roll_pitch_mean_deg=[statistics.mean(v) for v in angles[:2]],
                roll_pitch_stddev_deg=[statistics.pstdev(v) for v in angles[:2]],
                first_rpy_deg=[v[0] for v in angles], last_rpy_deg=[v[-1] for v in angles],
                relative_yaw_change_deg=yaw_change, absolute_yaw_valid=False,
                first_total_rpy_deg=[v[0] for v in totals], last_total_rpy_deg=[v[-1] for v in totals],
                total_kind="body",
                total_epochs=sorted({int(row["total_epoch"]) for row in usable}),
                final_residual_bias_rad_s=[float(usable[-1][k]) for k in ("bx", "by", "bz")],
                accel_weight_mean=statistics.mean(float(r["weight"]) for r in usable))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--calibration", type=Path, help="override log # CAL, or supply older logs' calibration")
    parser.add_argument("--native", type=Path, default=Path("build/host-debug/tests/host/attitude_replay.exe"))
    parser.add_argument("--output", type=Path, help="optional quaternion and Euler CSV")
    args = parser.parse_args()
    try:
        lines = args.input.read_text(encoding="utf-8-sig").splitlines(keepends=True)
        calibration = json.loads(args.calibration.read_text(encoding="utf-8-sig")) if args.calibration else calibration_from_log(lines)
        rows = list(imu.calibrated_records(imu.records(lines), calibration))
        text = native_replay(rows, args.native.resolve())
        report = summarize(text)
        report["input_imu"] = imu.summarize(rows)
        report["calibration_source"] = str(args.calibration) if args.calibration else "accepted # CAL (logged micro-units)"
        if args.output:
            args.output.write_text(text, encoding="utf-8")
        print(json.dumps(report, indent=2))
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"Attitude replay rejected: {error}\n")


if __name__ == "__main__":
    main()
