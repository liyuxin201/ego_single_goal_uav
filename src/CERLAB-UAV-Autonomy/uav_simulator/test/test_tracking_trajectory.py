#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from tracking_trajectory import Trajectory


def norm(vector):
    return math.sqrt(sum(component * component for component in vector))


class TrackingTrajectoryTest(unittest.TestCase):
    def test_waypoint_segment_uses_arc_length_speed(self):
        trajectory = Trajectory.from_config(
            {
                "loop": False,
                "segments": [
                    {
                        "type": "waypoint",
                        "speed": 2.0,
                        "points": [[0.0, 0.0, 0.0], [3.0, 4.0, 0.0]],
                    }
                ],
            }
        )

        sample = trajectory.sample(1.0)

        self.assertAlmostEqual(2.5, trajectory.duration)
        self.assertAlmostEqual(1.2, sample.position[0])
        self.assertAlmostEqual(1.6, sample.position[1])
        self.assertAlmostEqual(2.0, norm(sample.velocity))

    def test_sine_segment_keeps_resultant_speed_constant(self):
        trajectory = Trajectory.from_config(
            {
                "loop": False,
                "samples": 300,
                "segments": [
                    {
                        "type": "sine",
                        "speed": 1.5,
                        "start": [0.0, 0.0, 0.0],
                        "end": [10.0, 0.0, 0.0],
                        "amplitude": 1.0,
                        "cycles": 1.0,
                    }
                ],
            }
        )

        sample = trajectory.sample(trajectory.duration * 0.25)

        self.assertGreater(trajectory.duration, 10.0 / 1.5)
        self.assertGreater(sample.position[1], 0.5)
        self.assertAlmostEqual(1.5, norm(sample.velocity), places=3)

    def test_lane_change_segment_samples_mid_lateral_offset(self):
        trajectory = Trajectory.from_config(
            {
                "loop": False,
                "samples": 300,
                "segments": [
                    {
                        "type": "lane_change",
                        "speed": 1.0,
                        "start": [0.0, 0.0, 0.0],
                        "end": [10.0, 2.0, 0.0],
                        "smoothness": 1.0,
                    }
                ],
            }
        )

        sample = trajectory.sample(trajectory.duration * 0.5)

        self.assertAlmostEqual(1.0, sample.position[1], delta=0.12)
        self.assertAlmostEqual(1.0, norm(sample.velocity), places=3)

    def test_multi_segment_trajectory_advances_across_segments(self):
        trajectory = Trajectory.from_config(
            {
                "loop": False,
                "segments": [
                    {"type": "waypoint", "speed": 1.0, "points": [[0, 0, 0], [1, 0, 0]]},
                    {"type": "waypoint", "speed": 2.0, "points": [[1, 0, 0], [1, 4, 0]]},
                ],
            }
        )

        sample = trajectory.sample(2.0)

        self.assertAlmostEqual(1.0, sample.position[0])
        self.assertAlmostEqual(2.0, sample.position[1])
        self.assertAlmostEqual(2.0, norm(sample.velocity))

    def test_non_loop_trajectory_clamps_after_end(self):
        trajectory = Trajectory.from_config(
            {
                "loop": False,
                "segments": [
                    {"type": "waypoint", "speed": 1.0, "points": [[0, 0, 0], [1, 0, 0]]}
                ],
            }
        )

        sample = trajectory.sample(10.0)

        self.assertEqual((1.0, 0.0, 0.0), sample.position)
        self.assertEqual((0.0, 0.0, 0.0), sample.velocity)


if __name__ == "__main__":
    unittest.main()
