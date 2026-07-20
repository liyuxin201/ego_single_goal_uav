#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class Px4TakeoffTrackingLaunchTest(unittest.TestCase):
    def test_launch_starts_takeoff_hover_and_tracking_controller(self):
        launch_path = PACKAGE_DIR / "launch" / "px4_takeoff_tracking.launch"
        self.assertTrue(launch_path.exists())

        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("1.0", args["takeoff_height"])
        self.assertEqual("/mavros/setpoint_raw/local", args["external_offboard_setpoint_topic"])
        self.assertEqual("0.3", args["external_offboard_timeout"])
        self.assertEqual("true", args["passthrough_raw_setpoint"])
        self.assertEqual("full_state", args["raw_setpoint_mode"])
        self.assertEqual("0.8", args["velocity_height_position_p_x"])
        self.assertEqual("0.8", args["velocity_height_position_p_y"])
        self.assertEqual("1.0", args["velocity_height_position_p_z"])
        self.assertEqual("3.0", args["velocity_height_max_velocity_x"])
        self.assertEqual("3.0", args["velocity_height_max_velocity_y"])
        self.assertEqual("1.0", args["velocity_height_max_velocity_z"])

        nodes = {
            node.attrib.get("name"): (node.attrib.get("pkg"), node.attrib.get("type"))
            for node in root.findall("./node")
        }

        self.assertEqual(
            ("autonomous_flight", "takeoff_and_hover_node"),
            nodes["takeoff_and_hover_px4_node"],
        )
        self.assertEqual(
            ("tracking_controller", "tracking_controller_node"),
            nodes["tracking_controller_node"],
        )

    def test_takeoff_hover_only_yields_to_raw_setpoint(self):
        launch_path = PACKAGE_DIR / "launch" / "px4_takeoff_tracking.launch"
        root = ET.parse(launch_path).getroot()

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }

        self.assertEqual("true", params["/autonomous_flight/use_px4_offboard"])
        self.assertEqual("true", params["/autonomous_flight/enable_control_output"])
        self.assertEqual("false", params["/autonomous_flight/listen_click_goal"])
        self.assertEqual("", params["/autonomous_flight/external_pose_target_topic"])
        self.assertEqual(
            "$(arg external_offboard_setpoint_topic)",
            params["/autonomous_flight/external_offboard_setpoint_topic"],
        )
        self.assertEqual(
            "$(arg external_offboard_timeout)",
            params["/autonomous_flight/external_offboard_timeout"],
        )
        self.assertEqual("$(arg passthrough_raw_setpoint)", params["/controller/passthrough_raw_setpoint"])
        self.assertEqual("$(arg raw_setpoint_mode)", params["/controller/raw_setpoint_mode"])

        rosparams = {
            rosparam.attrib["param"]: (rosparam.text or "").strip()
            for rosparam in root.findall("./rosparam")
            if "param" in rosparam.attrib
        }
        self.assertEqual(
            "[$(arg velocity_height_position_p_x), $(arg velocity_height_position_p_y), $(arg velocity_height_position_p_z)]",
            rosparams["/controller/velocity_height_position_p"],
        )
        self.assertEqual(
            "[$(arg velocity_height_max_velocity_x), $(arg velocity_height_max_velocity_y), $(arg velocity_height_max_velocity_z)]",
            rosparams["/controller/velocity_height_max_velocity"],
        )

    def test_loads_tracking_controller_params(self):
        launch_path = PACKAGE_DIR / "launch" / "px4_takeoff_tracking.launch"
        root = ET.parse(launch_path).getroot()

        controller_rosparams = [
            rosparam
            for rosparam in root.findall("./rosparam")
            if rosparam.attrib.get("ns") == "controller"
        ]

        self.assertEqual(1, len(controller_rosparams))
        self.assertEqual(
            "$(find tracking_controller)/cfg/controller_param.yaml",
            controller_rosparams[0].attrib["file"],
        )


if __name__ == "__main__":
    unittest.main()
