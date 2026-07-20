#!/usr/bin/env python3

import subprocess
import sys
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


def node_by_name(launch_file, name):
    tree = launch_tree(launch_file)
    return tree.find(f".//node[@name='{name}']")


class TargetUavLaunchSplitTest(unittest.TestCase):
    def test_large_pillar_environment_launch_uses_local_px4_plugin_mid360_model(self):
        defaults = arg_defaults("large_pillar_start.launch")
        includes = include_args("large_pillar_start.launch")
        env = env_values("large_pillar_start.launch")

        self.assertEqual(
            "$(find uav_simulator)/worlds/large_pillar/large_pillar_sparse_dynamic.world",
            defaults["world_name"],
        )
        self.assertEqual(
            "$(find uav_simulator)/urdf/px4_iris_mid360.sdf",
            defaults["sdf_model"],
        )

        gazebo_args = includes["$(find gazebo_ros)/launch/empty_world.launch"]
        self.assertEqual("$(arg world_name)", gazebo_args["world_name"])
        self.assertIn("$(find mavros)/launch/px4.launch", includes)

        self.assertIn("$(find uav_simulator)/../PX4-SITL_gazebo-classic/models", env["GAZEBO_MODEL_PATH"])
        self.assertIn("$(find livox_laser_simulation)/models", env["GAZEBO_MODEL_PATH"])
        self.assertIn("$(find uav_simulator)/../PX4-SITL_gazebo-classic/build", env["GAZEBO_PLUGIN_PATH"])
        self.assertIn("$(find livox_laser_simulation)/../../../devel/lib", env["GAZEBO_PLUGIN_PATH"])

    def test_large_pillar_environment_launch_does_not_depend_on_px4_ros_package(self):
        tree = launch_tree("large_pillar_start.launch")
        xml = ET.tostring(tree.getroot(), encoding="unicode")

        self.assertNotIn("$(find px4)", xml)

    def test_large_pillar_environment_launch_starts_local_px4_sitl(self):
        defaults = arg_defaults("large_pillar_start.launch")
        sitl_node = node_by_name("large_pillar_start.launch", "sitl")

        self.assertIn("px4_root", defaults)
        self.assertIn("px4_sim_model", defaults)
        self.assertIn("px4_instance", defaults)
        self.assertEqual("$(find uav_simulator)/../PX4-Autopilot", defaults["px4_root"])
        self.assertEqual("gazebo-classic_iris", defaults["px4_sim_model"])
        self.assertEqual("0", defaults["px4_instance"])
        self.assertIsNotNone(sitl_node)
        self.assertEqual("uav_simulator", sitl_node.attrib["pkg"])
        self.assertEqual("start_px4_sitl.py", sitl_node.attrib["type"])
        self.assertIn("--px4-root $(arg px4_root)", sitl_node.attrib["args"])
        self.assertIn("--sim-model $(arg px4_sim_model)", sitl_node.attrib["args"])
        self.assertIn("--instance $(arg px4_instance)", sitl_node.attrib["args"])

    def test_large_pillar_environment_launch_does_not_start_non_px4_or_target_uav(self):
        names = node_names("large_pillar_start.launch")

        self.assertNotIn("spawn_gazebo_model", names)
        self.assertNotIn("keyboard_control", names)
        self.assertNotIn("spawn_target_uav_model", names)
        self.assertNotIn("target_uav_controller", names)

    def test_large_pillar_environment_launch_spawns_px4_plugin_model(self):
        tree = launch_tree("large_pillar_start.launch")
        spawn_node = tree.find(".//node[@name='spawn_px4_model']")

        self.assertIsNotNone(spawn_node)
        self.assertEqual("gazebo_ros", spawn_node.attrib["pkg"])
        self.assertEqual("spawn_model", spawn_node.attrib["type"])
        self.assertIn("-sdf -file $(arg sdf_model)", spawn_node.attrib["args"])
        self.assertIn("-model $(arg vehicle_model)", spawn_node.attrib["args"])

    def test_large_pillar_environment_launch_keeps_dynamic_pillar_state_publisher(self):
        names = node_names("large_pillar_start.launch")

        self.assertIn("dynamic_pillar_state_node", names)

    def test_target_uav_launch_starts_target_model_and_controller(self):
        names = node_names("large_pillar_target_uav.launch")

        self.assertIn("spawn_target_uav_model", names)
        self.assertIn("target_uav_controller", names)

    def test_target_uav_model_has_lidar_detectable_collision_geometry(self):
        tree = ET.parse(PACKAGE_DIR / "models" / "target_uav" / "model.sdf")
        collision = tree.find(".//collision[@name='lidar_detection_collision']")

        self.assertIsNotNone(collision)
        self.assertEqual("200", collision.findtext("laser_retro"))
        self.assertEqual("1.20 1.20 0.30", collision.findtext("geometry/box/size"))

    def test_local_px4_mid360_model_contains_px4_plugin_and_lidar(self):
        tree = ET.parse(PACKAGE_DIR / "urdf" / "px4_iris_mid360.sdf")

        self.assertIsNotNone(tree.find(".//plugin[@name='mavlink_interface']"))
        self.assertIsNotNone(tree.find(".//sensor[@name='laser_livox']"))
        self.assertEqual("/livox/lidar", tree.findtext(".//sensor[@name='laser_livox']/plugin/ros_topic"))

    def test_local_px4_sitl_wrapper_exists(self):
        wrapper = PACKAGE_DIR / "scripts" / "start_px4_sitl.py"

        self.assertTrue(wrapper.exists())

    def test_local_px4_sitl_wrapper_accepts_ros_remap_args(self):
        wrapper = PACKAGE_DIR / "scripts" / "start_px4_sitl.py"
        result = subprocess.run(
            [
                sys.executable,
                str(wrapper),
                "--px4-root",
                "/tmp/not-real-px4-root",
                "__name:=sitl",
                "__log:=/tmp/sitl.log",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        self.assertEqual(1, result.returncode)
        self.assertIn("PX4 SITL binary not found", result.stderr)
        self.assertNotIn("unrecognized arguments", result.stderr)

    def test_local_px4_sitl_wrapper_keeps_px4_stdin_open(self):
        wrapper = PACKAGE_DIR / "scripts" / "start_px4_sitl.py"

        self.assertIn("stdin=subprocess.PIPE", wrapper.read_text())

    def test_local_px4_autopilot_is_ignored_by_catkin_make(self):
        catkin_ignore = PACKAGE_DIR / ".." / "PX4-Autopilot" / "CATKIN_IGNORE"

        self.assertTrue(catkin_ignore.exists())


if __name__ == "__main__":
    unittest.main()
