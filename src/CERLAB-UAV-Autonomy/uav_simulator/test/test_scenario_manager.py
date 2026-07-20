#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR))

from scenario_manager import (
    as_bool,
    build_obstacle_record,
    cylinder_sdf,
    load_scenario_config,
    motion_start_allowed,
    motion_elapsed,
    sample_with_start_delay,
    project_target_position,
    resolve_scenario_path,
    uav_ready_from_inputs,
)
from tracking_trajectory import Trajectory
from tracking_trajectory import TrajectorySample


class ScenarioManagerTest(unittest.TestCase):
    def test_resolve_scenario_path_maps_short_name(self):
        path = resolve_scenario_path("scenario4a", PACKAGE_DIR / "config" / "tracking_scenarios")

        self.assertEqual("scenario4a_lateral_crossing.yaml", path.name)

    def test_load_scenario_config_reads_phase1_yaml(self):
        config = load_scenario_config("scenario1", PACKAGE_DIR / "config" / "tracking_scenarios")

        self.assertEqual("scenario1", config["scenario"]["id"])
        self.assertEqual("ground_target", config["target"]["type"])

    def test_project_target_position_uses_tracking_plane_height(self):
        self.assertEqual((2.0, -1.0, 1.5), project_target_position((2.0, -1.0, 0.0), 1.5))

    def test_cylinder_sdf_contains_collision_visual_and_dimensions(self):
        sdf = cylinder_sdf("obs_crossing", radius=0.35, height=1.8, static=False)

        self.assertIn("<model name=\"obs_crossing\">", sdf)
        self.assertIn("<static>false</static>", sdf)
        self.assertIn("<radius>0.35</radius>", sdf)
        self.assertIn("<length>1.8</length>", sdf)
        self.assertIn("<collision name=\"collision\">", sdf)

    def test_build_obstacle_record_uses_size_as_radius(self):
        sample = TrajectorySample(
            position=(1.0, 2.0, 0.0),
            velocity=(0.5, 0.0, 0.0),
            yaw=0.0,
        )
        config = {"name": "obs_1", "radius": 0.4, "height": 1.8}

        record = build_obstacle_record(config, sample)

        self.assertEqual("obs_1", record["name"])
        self.assertEqual(0.4, record["size"])
        self.assertEqual(1.8, record["height"])
        self.assertEqual((1.0, 2.0, 0.9), record["position"])
        self.assertEqual((0.5, 0.0, 0.0), record["velocity"])

    def test_uav_readiness_requires_altitude_arm_and_offboard(self):
        odom_low = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=0.2))))
        odom_ready = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=1.1))))
        state_manual = SimpleNamespace(armed=True, mode="MANUAL")
        state_offboard_disarmed = SimpleNamespace(armed=False, mode="OFFBOARD")
        state_ready = SimpleNamespace(armed=True, mode="OFFBOARD")

        self.assertFalse(uav_ready_from_inputs(True, odom_low, state_ready, 0.8, True, True))
        self.assertFalse(uav_ready_from_inputs(True, odom_ready, state_manual, 0.8, True, True))
        self.assertFalse(uav_ready_from_inputs(True, odom_ready, state_offboard_disarmed, 0.8, True, True))
        self.assertTrue(uav_ready_from_inputs(True, odom_ready, state_ready, 0.8, True, True))

    def test_motion_elapsed_freezes_until_ready_time(self):
        self.assertEqual(0.0, motion_elapsed(now_sec=12.0, motion_start_sec=None))
        self.assertEqual(2.5, motion_elapsed(now_sec=12.0, motion_start_sec=9.5))

    def test_motion_start_requires_uav_ready_and_start_signal(self):
        self.assertFalse(motion_start_allowed(uav_ready=False, wait_for_motion_start=True, motion_start_requested=True))
        self.assertFalse(motion_start_allowed(uav_ready=True, wait_for_motion_start=True, motion_start_requested=False))
        self.assertTrue(motion_start_allowed(uav_ready=True, wait_for_motion_start=True, motion_start_requested=True))
        self.assertTrue(motion_start_allowed(uav_ready=True, wait_for_motion_start=False, motion_start_requested=False))

    def test_sample_with_start_delay_holds_then_moves(self):
        trajectory = Trajectory.from_config(
            {
                "loop": False,
                "segments": [
                    {
                        "type": "waypoint",
                        "speed": 1.0,
                        "points": [[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]],
                    }
                ],
            }
        )

        before_start = sample_with_start_delay(trajectory, elapsed_time=1.0, start_delay=2.0)
        after_start = sample_with_start_delay(trajectory, elapsed_time=3.5, start_delay=2.0)

        self.assertEqual((0.0, 0.0, 0.0), before_start.position)
        self.assertEqual((0.0, 0.0, 0.0), before_start.velocity)
        self.assertEqual((1.5, 0.0, 0.0), after_start.position)
        self.assertEqual((1.0, 0.0, 0.0), after_start.velocity)

    def test_as_bool_parses_ros_launch_string_values(self):
        self.assertTrue(as_bool(True))
        self.assertTrue(as_bool("true"))
        self.assertTrue(as_bool("1"))
        self.assertFalse(as_bool(False))
        self.assertFalse(as_bool("false"))
        self.assertFalse(as_bool("0"))


if __name__ == "__main__":
    unittest.main()
