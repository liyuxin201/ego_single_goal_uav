#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class TakeoffAndHoverPx4LaunchTest(unittest.TestCase):
    def test_launch_exposes_external_offboard_handover_controls(self):
        launch_path = PACKAGE_DIR / "launch" / "takeoff_and_hover_px4.launch"
        self.assertTrue(launch_path.exists())

        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("false", args["listen_click_goal"])
        self.assertEqual("", args["external_pose_target_topic"])
        self.assertEqual("/mavros/setpoint_raw/local", args["external_offboard_setpoint_topic"])
        self.assertEqual("0.3", args["external_offboard_timeout"])

    def test_launch_wires_external_offboard_handover_params(self):
        launch_path = PACKAGE_DIR / "launch" / "takeoff_and_hover_px4.launch"
        root = ET.parse(launch_path).getroot()

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }

        self.assertEqual(
            "$(arg external_offboard_setpoint_topic)",
            params["/autonomous_flight/external_offboard_setpoint_topic"],
        )
        self.assertEqual(
            "$(arg external_offboard_timeout)",
            params["/autonomous_flight/external_offboard_timeout"],
        )


if __name__ == "__main__":
    unittest.main()
