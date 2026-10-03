"""Summarize P3 USB attitude frames and sampling/estimator diagnostics."""
import argparse
import json
import math
import statistics
from pathlib import Path


def fields(line):
    return dict(token.split("=", 1) for token in line.split() if "=" in token)


def summarize(lines):
    diagnostics = {tag: [] for tag in ("ATT", "AHRS", "STATS", "TEMP", "PIPE", "CAL")}
    for line in lines:
        if not line.endswith("\n"):
            continue
        for tag in diagnostics:
            if line.startswith(f"# {tag} "):
                diagnostics[tag].append(fields(line))
                break
    frames = diagnostics.pop("ATT")
    valid = [row for row in frames if row.get("valid") == "1"]
    if not valid:
        raise ValueError("no valid # ATT; check P3 firmware and calibration")
    angles = [[int(row["rpy_md"].split("/")[i])/1000 for row in valid] for i in range(3)]
    totals = [[int(row["total_rpy_md"].split("/")[i])/1000 for row in valid] for i in range(3)] if all("total_rpy_md" in row and row.get("total_kind")=="body" for row in valid) else None
    qnorms = [math.sqrt(sum((int(v)/1e6)**2 for v in row["q_u"].split("/"))) for row in valid]
    elapsed = (int(valid[-1]["t"])-int(valid[0]["t"]))/1e6
    yaw_change = sum((after-before+180)%360-180 for before, after in zip(angles[2], angles[2][1:]))
    deltas = {}
    for tag, keys in (("AHRS", ("updates", "reject", "timing", "missed", "timeout", "late")),
                      ("STATS", ("missed", "spierr", "overlap", "stale", "logdrop", "wait"))):
        records = diagnostics[tag]
        if len(records) >= 2:
            deltas[tag] = {key: [int(b)-int(a) for a, b in zip(records[0][key].split("/"), records[-1][key].split("/"))]
                           for key in keys}
    return dict(frames=len(frames), valid=len(valid), invalid=len(frames)-len(valid),
                elapsed_s=elapsed,
                update_rate_hz=((int(valid[-1]["seq"])-int(valid[0]["seq"]))%2**32)/elapsed if elapsed else None,
                quaternion_norm_error_max=max(abs(v-1) for v in qnorms),
                first_rpy_deg=[v[0] for v in angles], last_rpy_deg=[v[-1] for v in angles],
                first_total_rpy_deg=[v[0] for v in totals] if totals else None,
                last_total_rpy_deg=[v[-1] for v in totals] if totals else None,
                total_kind="body" if totals else None,
                total_epochs=sorted({int(row["total_epoch"]) for row in valid if "total_epoch" in row}),
                mean_rpy_deg=[statistics.mean(v) for v in angles],
                stddev_rpy_deg=[statistics.pstdev(v) for v in angles],
                range_rpy_deg=[[min(v), max(v)] for v in angles], relative_yaw_change_deg=yaw_change,
                runtime_us_max=max(int(row["run_us"]) for row in valid),
                latency_us_max=max(int(row["lat_us"]) for row in valid),
                absolute_yaw_valid=any(row["yaw_abs"] != "0" for row in valid),
                counter_deltas=deltas,
                diagnostics={tag: dict(first=rows[0], last=rows[-1]) for tag, rows in diagnostics.items() if rows})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(summarize(args.input.read_text(encoding="utf-8-sig").splitlines(keepends=True)), indent=2))
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f"Attitude analysis failed: {error}\n")


if __name__ == "__main__":
    main()
