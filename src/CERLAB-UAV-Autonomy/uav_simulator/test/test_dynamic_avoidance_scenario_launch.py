#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def launch_tree(launch_file):
    return ET.parse(PACKAGE_DIR / "launch" / launch_file)


def arg_defaults(launch_file):
    tree = launch_tree(launch_file)
    return {
        arg.attrib.get("name"): arg.attrib.get("default")
        for arg in tree.findall("./arg")
    }


def env_values(launch_file):
    tree = launch_tree(launch_file)
    return {
        env.attrib.get("name"): env.attrib.get("value")
        for env in tree.findall("./env")
    }


def include_args(launch_file):
    tree = launch_tree(launch_file)
    includes = {}
    for include in tree.findall(".//include"):
        includes[include.attrib.get("file")] = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in include.findall("./arg")
        }
    return includes


def node_names(launch_file):
    tree = launch_tree(launch_file)
    return {node.attrib.get("name") for node in tree.findall(".//node")}


class DynamicAvoidanceScenarioLaunchTest(unittest.TestCase):
    def test_dynamic_avoidance_scenario_launch_wires_non_px4_lidar_stack(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_avoidance_scenario.launch"
        self.assertTrue(launch_path.exists())

        defaults = arg_defaults("dynamic_avoidance_scenario.launch")
        env = env_values("dynamic_avoidance_scenario.launch")
        includes = include_args("dynamic_avoidance_scenario.launch")
        names = node_names("dynamic_avoidance_scenario.launch")

        self.assertEqual(
            "$(find uav_simulator)/worlds/large_pillar/large_pillar_dynamic_only.world",
            defaults["world_name"],
        )
        self.assertEqual(
            "$(find autonomous_flight)/launch/dynamic_navigation_lidar.launch",
            defaults["avoidance_launch"],
        )
        self.assertEqual("false", defaults["start_avoidance"])
        self.assertEqual("false", defaults["start_rviz"])
        self.assertEqual("-23.0", defaults["x"])
        self.assertEqual("0.1", defaults["z"])

        gazebo_args = includes["$(find gazebo_ros)/launch/empty_world.launch"]
        self.assertEqual("$(arg world_name)", gazebo_args["world_name"])
        self.assertIn("$(find uav_simulator)/launch/setupTF.launch", includes)
        self.assertIn("$(arg avoidance_launch)", includes)
        self.assertIn("$(find uav_simulator)/launch/rviz.launch", includes)

        self.assertIn("$(find livox_laser_simulation)/models", env["GAZEBO_MODEL_PATH"])
        self.assertIn("$(find uav_simulator)/plugins", env["GAZEBO_PLUGIN_PATH"])

        for required in [
            "gt_pose_throttle",
            "gt_vel_throttle",
            "gt_acc_throttle",
            "gt_odom_throttle",
            "spawn_gazebo_model",
        ]:
            self.assertIn(required, names)

        self.assertNotIn("keyboard_control", names)
        self.assertNotIn("sitl", names)

    def test_dynamic_only_world_keeps_ground_plane_and_removes_static_pillars(self):
        world_path = PACKAGE_DIR / "worlds" / "large_pillar" / "large_pillar_dynamic_only.world"
        self.assertTrue(world_path.exists())

        root = ET.parse(world_path).getroot()
        models = root.findall(".//world[@name='default']/model")
        static_model_names = {
            model.attrib["name"]
            for model in models
            if model.findtext("static") == "1"
        }
        dynamic_model_names = {
            model.attrib["name"]
            for model in models
            if model.attrib["name"].startswith("dynamic_")
        }

        self.assertEqual({"ground_plane"}, static_model_names)
        self.assertEqual(7, len(dynamic_model_names))


if __name__ == "__main__":
    unittest.main()
