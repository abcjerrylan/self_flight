"""Replay P2B ASCII CSV v1. Uses only the Python standard library."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def records(lines):
    for line in lines:
        # Capture can stop midway through its final row.
        if not line.startswith("IMU,") or not line.endswith("\n"):
            continue
        parts = line.strip().split(",")
        if len(parts) != 11 or parts[1] not in ("A", "G"):
            raise ValueError("invalid IMU CSV row")
        sequence, measured, started, available, x, y, z, sensor_time, valid = map(int, parts[2:])
        if not (0 <= sequence < 2**32 and 0 <= sensor_time < 2**24
                and all(-32768 <= v <= 32767 for v in (x, y, z)) and valid in (0, 1)
                and 0 <= measured <= started <= available < 2**64):
            raise ValueError("invalid IMU fields or timestamp order")
        scale = 12 * 9.80665 / 32768 if parts[1] == "A" else 2000 * math.pi / (180 * 32768)
        si = (x * scale, y * scale, z * scale)
        yield dict(sensor=parts[1], sequence=sequence, measured_us=measured, started_us=started,
                   available_us=available, raw=(x, y, z), sensor_time=sensor_time, valid=valid,
                   sensor_si=si, body_si=(-si[1], -si[0], -si[2]),
                   latency_us=available-measured, read_us=available-started)



def calibrated_records(rows, calibration):
    if (calibration.get("schema_version") != 1 or calibration.get("quality") != "Accepted"
            or calibration.get("board_identity") != 0x74323535 or calibration.get("sensor_identity") != 0xB0880F1E
            or calibration.get("axes") != "sensor"):
        raise ValueError("incompatible or unaccepted calibration")
    vectors = [calibration[key] for key in ("gyro_bias_rad_s", "accel_bias_m_s2", "accel_scale")]
    if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in vectors):
        raise ValueError("invalid calibration vector")
    gyro, bias, scale = vectors
    if (sum(v*v for v in gyro) > 0.05**2 or sum(v*v for v in bias) > 1.0
            or any(not 0.9 <= v <= 1.1 for v in scale)):
        raise ValueError("calibration parameters outside accepted range")
    for original in rows:
        row = dict(original)
        si = row["sensor_si"]
        corrected = tuple(si[i]-gyro[i] if row["sensor"] == "G" else (si[i]-bias[i])*scale[i] for i in range(3))
        row["corrected_sensor_si"] = corrected
        row["raw_body_si"] = row["body_si"]
        row["body_si"] = (-corrected[1], -corrected[0], -corrected[2])
        yield row


def summarize(rows):
    report = {}
    for source in ("A", "G"):
        data = [r for r in rows if r["sensor"] == source]
        usable = [r for r in data if r["valid"]]
        gaps = duplicates = backward = 0
        intervals = []
        for before, after in zip(data, data[1:]):
            step = (after["sequence"] - before["sequence"]) % 2**32
            if step == 0:
                duplicates += 1
            elif step >= 2**31:
                backward += 1
            else:
                gaps += step - 1
            dt = after["measured_us"] - before["measured_us"]
            if dt > 0:
                intervals.append(dt)
            else:
                backward += 1
        elapsed = data[-1]["measured_us"] - data[0]["measured_us"] if len(data) > 1 else 0
        count_delta = (data[-1]["sequence"] - data[0]["sequence"]) % 2**32 if data else 0
        report[source] = dict(rows=len(data), valid=len(usable), invalid=len(data)-len(usable),
                              sequence_gaps=gaps, duplicates=duplicates, backward=backward,
                              event_rate_hz=count_delta*1e6/elapsed if elapsed > 0 else None,
                              dt_us_min=min(intervals) if intervals else None,
                              dt_us_max=max(intervals) if intervals else None,
                              latency_us_max=max((r["latency_us"] for r in data), default=None),
                              read_us_max=max((r["read_us"] for r in data), default=None))
        if usable:
            report[source]["body_mean_si"] = [statistics.mean(r["body_si"][i] for r in usable) for i in range(3)]
            report[source]["body_stddev_si"] = [statistics.pstdev(r["body_si"][i] for r in usable) for i in range(3)]
            if source == "A":
                report[source]["mean_force_norm_m_s2"] = statistics.mean(math.sqrt(sum(v*v for v in r["body_si"])) for r in usable)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, help="optional SI CSV output")
    parser.add_argument("--calibration", type=Path, help="accepted sensor-axis calibration JSON")
    args = parser.parse_args()
    with args.input.open(encoding="utf-8-sig") as stream:
        rows = list(records(stream))
    if args.calibration:
        calibration = json.loads(args.calibration.read_text(encoding="utf-8-sig"))
        rows = list(calibrated_records(rows, calibration))
    if not rows:
        parser.error("no P2B IMU rows; check firmware initialization and DTR")
    if args.output:
        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["sensor", "sequence", "measured_us", "started_us", "available_us",
                             "raw_x", "raw_y", "raw_z", "sensor_time", "valid", "sensor_x_si",
                             "sensor_y_si", "sensor_z_si", "body_x_si", "body_y_si", "body_z_si"])
            for row in rows:
                writer.writerow([row["sensor"], row["sequence"], row["measured_us"], row["started_us"],
                                 row["available_us"], *row["raw"], row["sensor_time"], row["valid"],
                                 *row["sensor_si"], *row["body_si"]])
    print(json.dumps(summarize(rows), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
