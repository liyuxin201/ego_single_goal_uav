#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class DynamicNavigationFake2DLaunchTest(unittest.TestCase):
    def setUp(self):
        self.launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_fake2d.launch"
        self.rviz_path = PACKAGE_DIR / "rviz" / "dynamic_navigation_fake2d.rviz"

    def test_launch_uses_fake_detector_and_px4_odom_defaults(self):
        self.assertTrue(self.launch_path.exists())

        root = ET.parse(self.launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}
        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }

        self.assertEqual("/mavros/local_position/odom", args["odom_topic"])
        self.assertEqual("/mavros/local_position/pose", args["pose_topic"])
        self.assertEqual("true", params["/autonomous_flight/use_fake_detector"])
        self.assertEqual("true", params["/autonomous_flight/use_dynamic_obstacle_truth_topic"])
        self.assertEqual(
            "$(arg dynamic_obstacle_truth_topic)",
            params["/autonomous_flight/dynamic_obstacle_truth_topic"],
        )
        self.assertEqual(
            "$(arg dynamic_obstacle_truth_marker_topic)",
            params["/autonomous_flight/dynamic_obstacle_truth_marker_topic"],
        )
        self.assertEqual("false", params["/autonomous_flight/enable_control_output"])
        self.assertEqual("true", params["/autonomous_flight/listen_click_goal"])
        self.assertEqual("$(find autonomous_flight)/rviz/dynamic_navigation_fake2d.rviz", args["rviz_config"])
        self.assertEqual("/large_pillar/dynamic_obstacles", args["dynamic_obstacle_truth_topic"])
        self.assertEqual("/onboard_detector/GT_obstacle_bbox", args["dynamic_obstacle_truth_marker_topic"])

        rviz_nodes = [
            node for node in root.findall("./node")
            if node.attrib.get("pkg") == "rviz"
            and node.attrib.get("type") == "rviz"
        ]
        self.assertEqual(1, len(rviz_nodes))
        self.assertEqual("-d $(arg rviz_config)", rviz_nodes[0].attrib["args"])

    def test_launch_does_not_load_lidar_detector_parameters(self):
        self.assertTrue(self.launch_path.exists())

        root = ET.parse(self.launch_path).getroot()
        loaded_files = [
            rosparam.attrib.get("file", "")
            for rosparam in root.findall("./rosparam")
        ]

        self.assertFalse(any("dynamic_detector_lidar_param.yaml" in path for path in loaded_files))
        self.assertFalse(any("mapping_lidar_param.yaml" in path for path in loaded_files))
        self.assertTrue(any("fake_detector_param.yaml" in path for path in loaded_files))

    def test_launch_exposes_algorithm_tuning_controls(self):
        self.assertTrue(self.launch_path.exists())

        root = ET.parse(self.launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}
        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }

        self.assertEqual("1.5", args["desired_velocity"])
        self.assertEqual("1.5", args["desired_acceleration"])
        self.assertEqual("0.1", args["replan_time_for_dynamic_obstacles"])
        self.assertEqual("0.03", args["dynamic_collision_check_step"])
        self.assertEqual("2.0", args["dynamic_collision_prediction_time"])
        self.assertEqual("0.8", args["dynamic_collision_safety_margin"])
        self.assertEqual("0.7", args["bspline_distance_threshold_dynamic"])

        self.assertEqual("$(arg desired_velocity)", params["/autonomous_flight/desired_velocity"])
        self.assertEqual("$(arg desired_acceleration)", params["/autonomous_flight/desired_acceleration"])
        self.assertEqual(
            "$(arg replan_time_for_dynamic_obstacles)",
            params["/autonomous_flight/replan_time_for_dynamic_obstacles"],
        )
        self.assertEqual(
            "$(arg dynamic_collision_check_step)",
            params["/autonomous_flight/dynamic_collision_check_step"],
        )
        self.assertEqual(
            "$(arg dynamic_collision_prediction_time)",
            params["/autonomous_flight/dynamic_collision_prediction_time"],
        )
        self.assertEqual(
            "$(arg dynamic_collision_safety_margin)",
            params["/autonomous_flight/dynamic_collision_safety_margin"],
        )
        self.assertEqual(
            "$(arg bspline_distance_threshold_dynamic)",
            params["/bspline_traj/distance_threshold_dynamic"],
        )

    def test_rviz_shows_2d_algorithm_topics(self):
        self.assertTrue(self.rviz_path.exists())

        rviz_text = self.rviz_path.read_text()

        self.assertIn("/onboard_detector/GT_obstacle_bbox", rviz_text)
        self.assertIn("/dynamic_map/inflated_voxel_map", rviz_text)
        self.assertIn("/dynamicNavigation/bspline_trajectory", rviz_text)
        self.assertIn("/dynamicNavigation/input_trajectory", rviz_text)
        self.assertIn("/autonomous_flight/planner_pose_target", rviz_text)


if __name__ == "__main__":
    unittest.main()
