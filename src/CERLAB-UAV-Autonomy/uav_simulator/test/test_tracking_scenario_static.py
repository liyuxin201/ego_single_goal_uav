#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class TrackingScenarioStaticTest(unittest.TestCase):
    def test_planner_eval_diagnostics_message_declares_required_fields(self):
        msg_path = PACKAGE_DIR / "msg" / "PlannerEvalDiagnostics.msg"
        self.assertTrue(msg_path.exists())
        text = msg_path.read_text()

        for field in [
            "int32 replan_id",
            "bool success",
            "string planner_mode",
            "float64 total_time_ms",
            "float64 search_time_ms",
            "float64 optimize_time_ms",
            "float64 st_expand_time_ms",
            "int32 st_nodes_expanded",
            "int32 st_edges_checked",
            "int32 checked_obstacles",
            "int32 relevant_obstacles",
            "int32 pruned_dominated",
            "int32 pruned_total",
            "float64 recovery_time_est",
        ]:
            self.assertIn(field, text)

    def test_tracking_launch_exposes_expected_controls(self):
        launch_path = PACKAGE_DIR / "launch" / "tracking_scenario.launch"
        self.assertTrue(launch_path.exists())
        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}
        nodes = {(node.attrib.get("pkg"), node.attrib.get("type")) for node in root.findall(".//node")}

        self.assertEqual("scenario1", args["scenario"])
        self.assertEqual("true", args["start_gazebo"])
        self.assertEqual("true", args["start_px4"])
        self.assertEqual("true", args["start_mavros"])
        self.assertEqual("true", args["start_scenario_manager"])
        self.assertEqual("true", args["start_evaluator"])
        self.assertEqual("false", args["start_planner"])
        self.assertEqual("", args["planner_launch"])
        self.assertEqual("-9.5", args["x"])
        self.assertEqual("0.0", args["y"])
        self.assertEqual("0.0", args["z"])
        self.assertEqual("true", args["wait_for_motion_start"])
        self.assertEqual("/tracking_scenario/start_motion", args["motion_start_topic"])
        self.assertIn(("uav_simulator", "scenario_manager.py"), nodes)
        self.assertIn(("uav_simulator", "tracking_evaluator.py"), nodes)
        self.assertIn(("uav_simulator", "start_px4_sitl.py"), nodes)

    def test_cmake_builds_diagnostics_message_and_installs_scripts(self):
        cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text()

        self.assertIn("PlannerEvalDiagnostics.msg", cmake)
        self.assertIn("scripts/scenario_manager.py", cmake)
        self.assertIn("scripts/tracking_evaluator.py", cmake)

    def test_phase1_scenario_yaml_files_follow_schema(self):
        scenario_dir = PACKAGE_DIR / "config" / "tracking_scenarios"
        scenarios = {
            "scenario1_normal_tracking.yaml": "scenario1",
            "scenario2_adaptive_stspace.yaml": "scenario2",
            "scenario4a_lateral_crossing.yaml": "scenario4a",
        }

        for filename, scenario_id in scenarios.items():
            with self.subTest(filename=filename):
                path = scenario_dir / filename
                self.assertTrue(path.exists())
                data = yaml.safe_load(path.read_text())

                self.assertEqual(scenario_id, data["scenario"]["id"])
                self.assertIn("world", data["scenario"])
                self.assertEqual(1.5, data["scenario"]["z_track"])
                self.assertIn("initial_pose", data["uav"])
                self.assertEqual(0.0, data["uav"]["initial_pose"][2])
                self.assertEqual("ground_target", data["target"]["type"])
                self.assertEqual("/tmp/uav_tracking_eval", data["evaluation"]["output_dir"])
                self.assertIn("segments", data["target"]["trajectory"])
                self.assertIn("dynamic_obstacles", data)
                self.assertIn("static_obstacles_eval", data)

    def test_scenario1_starts_at_short_tracking_distance(self):
        path = PACKAGE_DIR / "config" / "tracking_scenarios" / "scenario1_normal_tracking.yaml"
        data = yaml.safe_load(path.read_text())

        self.assertEqual([-9.5, 0.0, 0.0, 0.0], data["uav"]["initial_pose"])
        first_target_point = data["target"]["trajectory"]["segments"][0]["points"][0]
        self.assertEqual([-8.0, 0.0, 0.0], first_target_point)

        tracking = data["evaluation"]["tracking"]
        self.assertEqual(1.5, tracking["desired_distance"])
        self.assertEqual(0.5, tracking["min_distance"])
        self.assertEqual(2.0, tracking["max_distance"])

    def test_scenario4a_uses_same_uav_spawn_as_scenario1(self):
        path = PACKAGE_DIR / "config" / "tracking_scenarios" / "scenario4a_lateral_crossing.yaml"
        data = yaml.safe_load(path.read_text())

        self.assertEqual([-9.5, 0.0, 0.0, 0.0], data["uav"]["initial_pose"])

    def test_scenario2_contains_multi_dynamic_obstacles_for_stspace(self):
        path = PACKAGE_DIR / "config" / "tracking_scenarios" / "scenario2_adaptive_stspace.yaml"
        data = yaml.safe_load(path.read_text())

        self.assertEqual([-9.5, 0.0, 0.0, 0.0], data["uav"]["initial_pose"])
        self.assertEqual("scenario2", data["scenario"]["id"])
        self.assertEqual(10, len(data["dynamic_obstacles"]))

        obstacle_names = {obstacle["name"] for obstacle in data["dynamic_obstacles"]}
        for name in ["obs_relevant_1", "obs_relevant_2", "obs_relevant_3"]:
            self.assertIn(name, obstacle_names)

        relevant = {
            obstacle["name"]: obstacle
            for obstacle in data["dynamic_obstacles"]
            if obstacle["name"].startswith("obs_relevant_")
        }
        self.assertLess(0.0, relevant["obs_relevant_1"]["trajectory"]["start_delay"])
        self.assertLess(relevant["obs_relevant_1"]["trajectory"]["start_delay"], relevant["obs_relevant_2"]["trajectory"]["start_delay"])
        self.assertLess(relevant["obs_relevant_2"]["trajectory"]["start_delay"], relevant["obs_relevant_3"]["trajectory"]["start_delay"])

        unrelated = [
            obstacle for obstacle in data["dynamic_obstacles"]
            if obstacle["name"].startswith("obs_unrelated_")
        ]
        self.assertEqual(7, len(unrelated))
        for obstacle in unrelated:
            points = obstacle["trajectory"]["segments"][0]["points"]
            self.assertGreaterEqual(min(point[1] for point in points), 6.0)

        static_names = {obstacle["name"] for obstacle in data["static_obstacles_eval"]}
        for name in ["wall_x_min", "wall_x_max", "wall_y_min", "wall_y_max"]:
            self.assertIn(name, static_names)

    def test_scenario1_contains_boundary_walls(self):
        path = PACKAGE_DIR / "config" / "tracking_scenarios" / "scenario1_normal_tracking.yaml"
        data = yaml.safe_load(path.read_text())
        obstacles = {obstacle["name"]: obstacle for obstacle in data["static_obstacles_eval"]}

        for name in ["wall_x_min", "wall_x_max", "wall_y_min", "wall_y_max"]:
            with self.subTest(name=name):
                self.assertIn(name, obstacles)
                self.assertEqual("box", obstacles[name]["type"])
                self.assertEqual(3.0, obstacles[name]["size"][2])

        self.assertEqual([-12.125, 0.0, 1.5], obstacles["wall_x_min"]["center"])
        self.assertEqual([20.125, 0.0, 1.5], obstacles["wall_x_max"]["center"])
        self.assertEqual([4.0, -8.125, 1.5], obstacles["wall_y_min"]["center"])
        self.assertEqual([4.0, 8.125, 1.5], obstacles["wall_y_max"]["center"])

    def test_base_empty_world_exists_with_ground_plane(self):
        world_path = PACKAGE_DIR / "worlds" / "tracking" / "base_empty.world"
        self.assertTrue(world_path.exists())
        root = ET.parse(world_path).getroot()

        self.assertIsNotNone(root.find(".//world[@name='default']"))
        self.assertIsNotNone(root.find(".//model[@name='ground_plane']"))


if __name__ == "__main__":
    unittest.main()
