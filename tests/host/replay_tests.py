import importlib.util
import math
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location("replay", Path(__file__).parents[2] / "tools/replay-imu.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class ReplayTests(unittest.TestCase):
    def test_wrap_gap_and_units(self):
        rows = list(replay.records([
            "# BMI088 init=1\n",
            "IMU,G,4294967295,4294967000,4294967010,4294967020,16384,0,0,0,1\n",
            "IMU,G,0,4294968000,4294968010,4294968020,0,0,0,0,1\n",
            "IMU,G,2,4294970000,4294970010,4294970020,0,0,0,0,0\n",
            "IMU,A,1,4294967000,4294967010,4294967020,0,0,2731,16777215,1\n",
        ]))
        report = replay.summarize(rows)
        self.assertEqual(report["G"]["sequence_gaps"], 1)
        self.assertEqual(report["G"]["invalid"], 1)
        self.assertEqual(report["G"]["event_rate_hz"], 1000)
        self.assertEqual(report["G"]["read_us_max"], 10)
        self.assertAlmostEqual(rows[0]["body_si"][1], -1000*math.pi/180)
        self.assertLess(rows[3]["body_si"][2], -9.8)

    def test_reject_bad_time_and_raw(self):
        for row in ["IMU,G,1,100,99,110,0,0,0,0,1\n", "IMU,A,1,100,101,110,40000,0,0,0,1\n"]:
            with self.assertRaises(ValueError):
                list(replay.records([row]))

    def test_calibration_before_mounting(self):
        row = list(replay.records(["IMU,A,1,100,101,110,0,0,2731,0,1\n"]))[0]
        calibration = dict(schema_version=1, quality="Accepted", board_identity=0x74323535,
                           sensor_identity=0xB0880F1E, axes="sensor", gyro_bias_rad_s=[0,0,0],
                           accel_bias_m_s2=[0.1,0.2,0.3], accel_scale=[1.01,0.99,1.02])
        corrected = list(replay.calibrated_records([row],calibration))[0]
        self.assertAlmostEqual(corrected["body_si"][0],0.198)
        self.assertAlmostEqual(corrected["body_si"][1],0.101)
        self.assertEqual(corrected["sensor_si"],row["sensor_si"])
        calibration["quality"]="Unknown"
        with self.assertRaises(ValueError):
            list(replay.calibrated_records([row],calibration))

    def test_partial_capture_tail(self):
        rows = list(replay.records(["IMU,G,1,100,101,110,0,0,0,0,1\n", "IMU,A,2,102"]))
        self.assertEqual(len(rows), 1)
        with self.assertRaises(ValueError):
            list(replay.records(["IMU,A,2,102\n"]))


if __name__ == "__main__":
    unittest.main()
