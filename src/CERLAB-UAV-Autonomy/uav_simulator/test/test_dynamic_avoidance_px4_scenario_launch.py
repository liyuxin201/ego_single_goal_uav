#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class DynamicAvoidancePx4ScenarioLaunchTest(unittest.TestCase):
    def test_target_uav_is_disabled_by_default(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_avoidance_px4_scenario.launch"
        self.assertTrue(launch_path.exists())

        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("false", args["start_target_uav"])

    def test_px4_scenario_uses_lidar360_model_by_default(self):
        start_path = PACKAGE_DIR / "launch" / "large_pillar_start.launch"
        self.assertTrue(start_path.exists())

        root = ET.parse(start_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("px4_iris_lidar360", args["vehicle_model"])
        self.assertEqual(
            "$(find uav_simulator)/urdf/px4_iris_lidar360.sdf",
            args["sdf_model"],
        )

    def test_lidar360_model_publishes_full_azimuth_pointcloud(self):
        sdf_path = PACKAGE_DIR / "urdf" / "px4_iris_lidar360.sdf"
        self.assertTrue(sdf_path.exists())

        root = ET.parse(sdf_path).getroot()
        horizontal = root.find(".//sensor[@name='laser_livox']/plugin/ray/scan/horizontal")
        vertical = root.find(".//sensor[@name='laser_livox']/plugin/ray/scan/vertical")
        plugin = root.find(".//sensor[@name='laser_livox']/plugin")

        self.assertIsNotNone(horizontal)
        self.assertIsNotNone(vertical)
        self.assertAlmostEqual(-3.14159, float(horizontal.findtext("min_angle")), places=5)
        self.assertAlmostEqual(3.14159, float(horizontal.findtext("max_angle")), places=5)
        self.assertEqual("40", vertical.findtext("samples"))
        self.assertAlmostEqual(-0.514872, float(vertical.findtext("min_angle")), places=6)
        self.assertAlmostEqual(0.514872, float(vertical.findtext("max_angle")), places=6)
        self.assertEqual("20000", plugin.findtext("samples"))
        self.assertEqual("lidar360-uniform.csv", plugin.findtext("csv_file_name"))
        self.assertEqual("/livox/lidar", plugin.findtext("ros_topic"))


if __name__ == "__main__":
    unittest.main()
