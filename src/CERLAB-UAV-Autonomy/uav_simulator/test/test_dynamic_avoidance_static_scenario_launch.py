#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class DynamicAvoidanceStaticScenarioLaunchTest(unittest.TestCase):
    def test_launch_selects_static_dynamic_world(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_avoidance_static_scenario.launch"
        self.assertTrue(launch_path.exists())

        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("true", args["gui"])
        self.assertEqual("false", args["start_avoidance"])
        self.assertEqual("false", args["start_rviz"])
        self.assertIn("large_pillar_sparse_dynamic.world", args["world_name"])

    def test_launch_forwards_world_to_base_scenario(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_avoidance_static_scenario.launch"
        root = ET.parse(launch_path).getroot()

        includes = root.findall("./include")
        self.assertEqual(1, len(includes))

        include = includes[0]
        self.assertIn("dynamic_avoidance_scenario.launch", include.attrib.get("file", ""))

        include_args = {
            arg.attrib["name"]: arg.attrib["value"]
            for arg in include.findall("./arg")
        }
        self.assertEqual("$(arg world_name)", include_args["world_name"])
        self.assertEqual("$(find uav_simulator)/worlds/large_pillar/large_pillar_sparse_dynamic.world", args := {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}["world_name"])


if __name__ == "__main__":
    unittest.main()
