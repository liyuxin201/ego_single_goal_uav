#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class DynamicAvoidancePx4StaticScenarioLaunchTest(unittest.TestCase):
    def test_launch_enables_static_obstacles(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_avoidance_px4_static_scenario.launch"
        self.assertTrue(launch_path.exists())

        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("true", args["gui"])
        self.assertEqual("true", args["publish_static_obstacles"])
        self.assertEqual("false", args["start_target_uav"])
        self.assertIn("large_pillar_sparse_dynamic.world", args["world_name"])

    def test_launch_uses_static_dynamic_world_in_startup(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_avoidance_px4_static_scenario.launch"
        root = ET.parse(launch_path).getroot()

        includes = [
            node.attrib.get("file")
            for node in root.findall("./include")
        ]

        self.assertTrue(any("large_pillar_start.launch" in include for include in includes))
        self.assertTrue(any("dynamic_avoidance_px4_scenario.launch" not in include for include in includes))

        start_launch = [
            node for node in root.findall("./include")
            if "large_pillar_start.launch" in node.attrib.get("file", "")
        ][0]
        start_args = {
            arg.attrib["name"]: arg.attrib["value"]
            for arg in start_launch.findall("./arg")
        }
        self.assertEqual("$(arg gui)", start_args["gui"])
        self.assertEqual("$(arg publish_static_obstacles)", start_args["publish_static_obstacles"])
        self.assertEqual("$(arg world_name)", start_args["world_name"])


if __name__ == "__main__":
    unittest.main()
