#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from target_uav_controller import WaypointPath, parse_waypoints


class TargetUavTrajectoryTest(unittest.TestCase):
    def test_parse_waypoints_accepts_semicolon_separated_xyz_triplets(self):
        waypoints = parse_waypoints("-18,0,1.4; -12 4 1.6; -6,10,1.4")

        self.assertEqual(waypoints, [(-18.0, 0.0, 1.4), (-12.0, 4.0, 1.6), (-6.0, 10.0, 1.4)])

    def test_sample_non_loop_path_clamps_to_last_waypoint(self):
        path = WaypointPath([(0.0, 0.0, 1.0), (3.0, 0.0, 1.0)], speed=1.5, loop=False)

        sample = path.sample(10.0)

        self.assertEqual(sample.position, (3.0, 0.0, 1.0))
        self.assertEqual(sample.velocity, (0.0, 0.0, 0.0))
        self.assertAlmostEqual(sample.yaw, 0.0)

    def test_sample_loop_path_wraps_and_uses_segment_velocity(self):
        path = WaypointPath(
            [(0.0, 0.0, 1.0), (2.0, 0.0, 1.0), (2.0, 2.0, 1.0)],
            speed=1.0,
            loop=True,
        )

        sample = path.sample(2.5)

        self.assertAlmostEqual(sample.position[0], 2.0)
        self.assertAlmostEqual(sample.position[1], 0.5)
        self.assertAlmostEqual(sample.position[2], 1.0)
        self.assertAlmostEqual(sample.velocity[0], 0.0)
        self.assertAlmostEqual(sample.velocity[1], 1.0)
        self.assertAlmostEqual(sample.velocity[2], 0.0)
        self.assertAlmostEqual(sample.yaw, math.pi / 2.0)


if __name__ == "__main__":
    unittest.main()
