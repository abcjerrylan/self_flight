import argparse
import csv
import io
import importlib.util
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("attitude_replay", ROOT/"tools/replay-attitude.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
parser = argparse.ArgumentParser()
parser.add_argument("--native", type=Path, default=ROOT/"build/host-debug/tests/host/attitude_replay.exe")
args = parser.parse_args()


class ReplayTests(unittest.TestCase):
    def test_logged_calibration(self):
        c = replay.calibration_from_log(["# CAL gyro=0 accel=1 quality=0\n",
            "# CAL gyro=1 accel=1 quality=2 bias_u=1000/-2000/3000 accel_bias_u=10000/20000/30000 scale_u=1000000/1001000/999000\n"])
        self.assertEqual(c["gyro_bias_rad_s"], [0.001, -0.002, 0.003])
        self.assertEqual(c["accel_scale"], [1, 1.001, 0.999])
        with self.assertRaises(ValueError):
            replay.calibration_from_log(["# CAL gyro=0 accel=1 quality=0\n"])

    def test_old_native_totals_rejected(self):
        with self.assertRaisesRegex(ValueError,"yaw-only"):
            replay.summarize("valid\n1\n")

    def test_native_tilt(self):
        rows = []
        force = -9.80665/math.sqrt(2)
        for n in range(100):
            for sensor in ("A", "G"):
                rows.append(dict(sensor=sensor, sequence=n, measured_us=n*1000,
                    available_us=n*1000+30, valid=1, body_si=(0, force, force) if sensor == "A" else (0, 0, 0)))
        report = replay.summarize(replay.native_replay(rows, args.native.resolve()))
        self.assertEqual(report["valid"], 100)
        self.assertAlmostEqual(report["last_rpy_deg"][0], 45, places=3)
        self.assertLess(report["quaternion_norm_error_max"], 1e-6)
        self.assertFalse(report["absolute_yaw_valid"])
        self.assertAlmostEqual(report["last_total_yaw_deg"],0,places=6)
        self.assertEqual(report["total_epochs"],[1])

    def test_missing_gyro(self):
        rows = []
        for n in (0, 1, 3, 4):
            for sensor in ("A", "G"):
                rows.append(dict(sensor=sensor, sequence=n, measured_us=n*1000,
                    available_us=n*1000+30, valid=1, body_si=(0, 0, -9.80665) if sensor == "A" else (0, 0, 0)))
        report = replay.summarize(replay.native_replay(rows, args.native.resolve()))
        self.assertEqual(report["invalid"], 1)
        self.assertEqual(report["error_counts"]["3"], 1)
        self.assertEqual(report["last_rpy_deg"], [0, 0, 0])
        self.assertEqual(report["total_epochs"],[1,2])

    def test_native_yaw_totals(self):
        rows=[]
        for n in range(2001):
            for sensor in ("A","G"):
                rows.append(dict(sensor=sensor,sequence=n,measured_us=n*1000,
                    available_us=n*1000+30,valid=1,body_si=(0,0,-9.80665) if sensor=="A" else (0,0,2*math.pi)))
        report=replay.summarize(replay.native_replay(rows,args.native.resolve()))
        self.assertEqual(report["valid"],2001)
        self.assertAlmostEqual(report["last_total_yaw_deg"],720,delta=.02)
        self.assertEqual(report["total_epochs"],[1])

    def test_native_tilted_yaw_feedback(self):
        rows=[]
        tilt=.7
        for n in range(1001):
            angle=n*.001
            accel=(9.80665*math.sin(tilt)*math.cos(angle),
                   -9.80665*math.sin(tilt)*math.sin(angle),-9.80665*math.cos(tilt))
            for sensor in ("A","G"):
                rows.append(dict(sensor=sensor,sequence=n,measured_us=n*1000,
                    available_us=n*1000+30,valid=1,body_si=accel if sensor=="A" else (0,0,1)))
        text=replay.native_replay(rows,args.native.resolve())
        samples=list(csv.DictReader(io.StringIO(text)))
        self.assertEqual(len(samples),1001)
        self.assertTrue(all(abs(float(row["total_yaw_deg"])-float(row["yaw_deg"]))<.001 for row in samples))
        self.assertGreater(abs(float(samples[-1]["total_yaw_deg"])-math.degrees(1)),2)
        self.assertNotIn("total_roll_deg",samples[-1])
        self.assertNotIn("total_pitch_deg",samples[-1])


if __name__ == "__main__":
    unittest.main(argv=[__file__])
