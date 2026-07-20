#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from tracking_evaluator import (
    build_sample_row,
    dynamic_obstacle_clearance,
    optional_message_type,
    planner_missing_defaults,
    static_obstacle_clearance,
    target_clearance,
    tracking_metrics,
    uav_world_position_from_mavros,
)


class TrackingEvaluatorTest(unittest.TestCase):
    def test_uav_world_position_adds_yaml_initial_pose_to_mavros_local_odom(self):
        position = uav_world_position_from_mavros(
            mavros_local_position=(0.2, -0.1, 0.0),
            uav_initial_pose=(-11.0, -3.0, 1.5, 0.0),
        )

        self.assertEqual((-10.8, -3.1, 1.5), position)

    def test_tracking_metrics_project_ground_target_to_tracking_height(self):
        metrics = tracking_metrics(
            uav_position=(0.0, 0.0, 1.5),
            uav_velocity=(0.8, 0.0, 0.0),
            target_position=(3.0, 0.0, 0.0),
            target_velocity=(0.8, 0.0, 0.0),
            z_track=1.5,
            desired_distance=3.0,
            min_distance=2.0,
            max_distance=4.5,
            max_relative_speed=1.0,
        )

        self.assertEqual((3.0, 0.0, 1.5), metrics["target_track"])
        self.assertAlmostEqual(3.0, metrics["tracking_distance"])
        self.assertAlmostEqual(0.0, metrics["tracking_error"])
        self.assertFalse(metrics["target_distance_violation"])
        self.assertFalse(metrics["relative_speed_violation"])

    def test_dynamic_obstacle_clearance_uses_sphere_distance(self):
        clearance = dynamic_obstacle_clearance(
            uav_position=(0.0, 0.0, 1.5),
            obstacle_position=(0.7, 0.0, 1.5),
            uav_radius=0.35,
            obstacle_radius=0.35,
        )

        self.assertAlmostEqual(0.0, clearance)

    def test_target_clearance_uses_projected_target_position(self):
        clearance = target_clearance(
            uav_position=(0.5, 0.0, 1.5),
            target_position=(0.0, 0.0, 0.0),
            z_track=1.5,
            uav_radius=0.35,
            target_radius=0.35,
        )

        self.assertLess(clearance, 0.0)

    def test_static_cylinder_clearance_uses_xy_distance_and_height(self):
        obstacle = {
            "type": "cylinder",
            "position": [0.0, 0.0, 0.0],
            "radius": 0.35,
            "height": 3.0,
        }

        clearance = static_obstacle_clearance((0.7, 0.0, 1.5), obstacle, uav_radius=0.35)

        self.assertAlmostEqual(0.0, clearance)

    def test_planner_missing_defaults_are_csv_safe(self):
        defaults = planner_missing_defaults()

        self.assertEqual(-1, defaults["planner_replan_id"])
        self.assertFalse(defaults["planner_success"])
        self.assertEqual("missing", defaults["planner_mode"])
        self.assertTrue(math.isnan(defaults["total_time_ms"]))
        self.assertEqual(-1, defaults["st_nodes_expanded"])

    def test_optional_message_type_returns_none_when_message_is_not_generated(self):
        def missing_import():
            raise ImportError("message not generated")

        self.assertIsNone(optional_message_type(missing_import))

    def test_build_sample_row_keeps_trajectory_length_nan_without_planning_trajectory(self):
        row = build_sample_row(
            run_id="run_a",
            scenario_id="scenario1",
            time_sec=1.25,
            uav_position=(0.0, 0.0, 1.5),
            uav_velocity=(0.0, 0.0, 0.0),
            target_position=(3.0, 0.0, 0.0),
            target_velocity=(0.0, 0.0, 0.0),
            z_track=1.5,
            tracking_config={
                "desired_distance": 3.0,
                "min_distance": 2.0,
                "max_distance": 4.5,
                "max_relative_speed": 1.0,
            },
            evaluation_config={
                "uav_radius": 0.35,
                "target_radius": 0.35,
                "collision_margin": 0.05,
            },
            dynamic_obstacles=[],
            static_obstacles=[],
            planner_diag=None,
            trajectory_length=math.nan,
        )

        self.assertEqual("run_a", row["run_id"])
        self.assertEqual("scenario1", row["scenario_id"])
        self.assertEqual("missing", row["planner_mode"])
        self.assertTrue(math.isnan(row["trajectory_length"]))
        self.assertAlmostEqual(0.0, row["tracking_error"])


if __name__ == "__main__":
    unittest.main()
