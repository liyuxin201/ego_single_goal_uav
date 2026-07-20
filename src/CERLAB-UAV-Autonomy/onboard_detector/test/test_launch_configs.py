#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def launch_tree(launch_file):
    return ET.parse(PACKAGE_DIR / "launch" / launch_file)


def arg_defaults(launch_file):
    tree = launch_tree(launch_file)
    return {
        arg.attrib.get("name"): arg.attrib.get("default")
        for arg in tree.findall("./arg")
    }


class OnboardDetectorLaunchConfigTest(unittest.TestCase):
    def test_run_lidar_detector_launch_matches_dynamic_avoidance_topics(self):
        launch_path = PACKAGE_DIR / "launch" / "run_lidar_detector.launch"
        self.assertTrue(launch_path.exists())

        defaults = arg_defaults("run_lidar_detector.launch")
        xml = launch_path.read_text()

        self.assertEqual("false", defaults["open_rviz"])
        self.assertEqual(
            "$(find onboard_detector)/rviz/lidar_detector_sim.rviz",
            defaults["rviz_config"],
        )
        self.assertIn("lidar_detector_param.yaml", xml)
        self.assertIn('if="$(arg open_rviz)"', xml)

        params = yaml.safe_load((PACKAGE_DIR / "cfg" / "lidar_detector_param.yaml").read_text())
        self.assertEqual("/livox/lidar", params["point_cloud_topic"])
        self.assertEqual("/CERLAB/quadcopter/pose", params["pose_topic"])
        self.assertEqual("/CERLAB/quadcopter/odom", params["odom_topic"])

    def test_lidar_detector_accepts_any_dynamic_box_under_three_meters(self):
        params = yaml.safe_load((PACKAGE_DIR / "cfg" / "lidar_detector_param.yaml").read_text())

        self.assertFalse(params["constrain_size"])
        self.assertEqual([3.0, 3.0, 3.0], params["max_dynamic_bbox_size"])

    def test_lidar_detector_uses_far_range_dynamic_profile(self):
        params = yaml.safe_load((PACKAGE_DIR / "cfg" / "lidar_detector_param.yaml").read_text())

        self.assertEqual(1, params["voxel_occupied_thresh"])
        self.assertEqual(4, params["dbscan_min_points_cluster"])
        self.assertEqual(0.35, params["dbscan_search_range_epsilon"])
        self.assertEqual(0.08, params["dynamic_velocity_threshold"])
        self.assertEqual(0.08, params["dynamic_center_velocity_threshold"])
        self.assertEqual(0.1, params["dynamic_voting_threshold"])
        self.assertEqual(0.75, params["maximum_skip_ratio"])
        self.assertEqual(3, params["kalman_filter_averaging_frames"])


if __name__ == "__main__":
    unittest.main()
