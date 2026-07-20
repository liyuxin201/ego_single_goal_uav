#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]


class DynamicNavigationLidarLaunchTest(unittest.TestCase):
    def test_launch_exposes_auto_goal_controls(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        self.assertTrue(launch_path.exists())

        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("false", args["auto_goal"])
        self.assertEqual("false", args["auto_takeoff"])
        self.assertEqual("0.45", args["planner_pose_target_lookahead"])
        self.assertEqual("0.5", args["control_target_lookahead"])
        self.assertEqual("true", args["enable_control_output"])
        self.assertEqual("false", args["publish_px4_raw_setpoint"])
        self.assertEqual("/mavros/setpoint_raw/local", args["px4_raw_setpoint_topic"])
        self.assertEqual("8.0", args["goal_x"])
        self.assertEqual("0.0", args["goal_y"])
        self.assertEqual("1.0", args["goal_z"])
        self.assertEqual("0.0", args["goal_yaw"])
        self.assertEqual("0.8", args["goal_ready_z"])
        self.assertEqual("1.0", args["goal_delay"])

    def test_launch_exposes_speed_safety_and_yaw_tuning_controls(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        expected_args = {
            "desired_velocity": "1.8",
            "desired_acceleration": "2.5",
            "desired_angular_velocity": "1.6",
            "replan_time_for_dynamic_obstacles": "0.03",
            "dynamic_collision_check_step": "0.005",
            "dynamic_collision_prediction_time": "4.0",
            "dynamic_collision_safety_margin": "2.5",
            "dynamic_collision_replan_distance": "0.0",
            "planner_pose_target_lookahead": "0.45",
            "control_target_lookahead": "0.5",
            "use_yaw_control": "true",
            "no_yaw_turning": "false",
            "robot_size_x": "0.5",
            "robot_size_y": "0.5",
            "robot_size_z": "0.3",
            "bspline_distance_threshold": "0.7",
            "bspline_distance_threshold_dynamic": "1.2",
            "bspline_weight_feasibility": "1.0",
            "bspline_weight_dynamic_obstacle": "5.0",
            "poly_initial_radius": "0.6",
            "poly_constraint_radius": "0.6",
        }

        for name, expected in expected_args.items():
            self.assertEqual(expected, args[name])

    def test_launch_starts_auto_goal_node(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()

        goal_nodes = [
            node for node in root.findall("./node")
            if node.attrib.get("pkg") == "autonomous_flight"
            and node.attrib.get("type") == "publish_nav_goal.py"
        ]

        self.assertEqual(1, len(goal_nodes))
        goal_node = goal_nodes[0]
        self.assertEqual("$(arg auto_goal)", goal_node.attrib.get("if"))

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in goal_node.findall("./param")
        }
        self.assertEqual("$(arg odom_topic)", params["odom_topic"])
        self.assertEqual("$(arg goal_x)", params["goal_x"])
        self.assertEqual("$(arg goal_y)", params["goal_y"])
        self.assertEqual("$(arg goal_z)", params["goal_z"])
        self.assertEqual("$(arg goal_yaw)", params["goal_yaw"])
        self.assertEqual("$(arg goal_ready_z)", params["ready_z"])
        self.assertEqual("$(arg goal_delay)", params["delay"])

    def test_launch_isolates_planner_from_px4_controller_params(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }

        self.assertEqual("$(arg auto_takeoff)", params["/autonomous_flight/auto_takeoff"])
        self.assertEqual("false", params["/autonomous_flight/release_control_on_goal"])
        self.assertEqual("", params["/autonomous_flight/external_pose_target_topic"])
        self.assertEqual(
            "$(arg planner_pose_target_lookahead)",
            params["/autonomous_flight/planner_pose_target_lookahead"],
        )
        self.assertEqual(
            "$(arg control_target_lookahead)",
            params["/autonomous_flight/control_target_lookahead"],
        )
        self.assertEqual("$(arg enable_control_output)", params["/autonomous_flight/enable_control_output"])
        self.assertEqual("$(arg use_yaw_control)", params["/autonomous_flight/use_yaw_control"])
        self.assertEqual("$(arg no_yaw_turning)", params["/autonomous_flight/no_yaw_turning"])

    def test_launch_enables_px4_raw_setpoint_output(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }

        self.assertEqual(
            "$(arg publish_px4_raw_setpoint)",
            params["/autonomous_flight/publish_px4_raw_setpoint"],
        )
        self.assertEqual(
            "$(arg px4_raw_setpoint_topic)",
            params["/autonomous_flight/px4_raw_setpoint_topic"],
        )

    def test_launch_wires_speed_safety_and_map_params(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }
        rosparams = {
            rosparam.attrib["param"]: (rosparam.text or "").strip()
            for rosparam in root.findall("./rosparam")
            if "param" in rosparam.attrib
        }

        self.assertEqual("$(arg desired_velocity)", params["/autonomous_flight/desired_velocity"])
        self.assertEqual("$(arg desired_acceleration)", params["/autonomous_flight/desired_acceleration"])
        self.assertEqual("$(arg desired_angular_velocity)", params["/autonomous_flight/desired_angular_velocity"])
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
            "$(arg dynamic_collision_replan_distance)",
            params["/autonomous_flight/dynamic_collision_replan_distance"],
        )
        self.assertEqual("$(arg map_resolution)", params["/rrt/map_resolution"])
        self.assertEqual("$(arg map_resolution)", params["/dynamic_map/map_resolution"])
        self.assertEqual("$(arg bspline_distance_threshold)", params["/bspline_traj/distance_threshold"])
        self.assertEqual("$(arg bspline_distance_threshold_dynamic)", params["/bspline_traj/distance_threshold_dynamic"])
        self.assertEqual("$(arg bspline_weight_feasibility)", params["/bspline_traj/weight_feasibility"])
        self.assertEqual("$(arg bspline_weight_dynamic_obstacle)", params["/bspline_traj/weight_dynamic_obstacle"])
        self.assertEqual("$(arg poly_initial_radius)", params["/poly_traj/initial_radius"])
        self.assertEqual("$(arg poly_constraint_radius)", params["/poly_traj/constraint_radius"])
        self.assertEqual(
            "[$(arg robot_size_x), $(arg robot_size_y), $(arg robot_size_z)]",
            rosparams["/dynamic_map/robot_size"],
        )

    def test_launch_starts_tracking_controller(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()

        tracking_nodes = [
            node for node in root.findall("./node")
            if node.attrib.get("pkg") == "tracking_controller"
            and node.attrib.get("type") == "tracking_controller_node"
        ]

        self.assertEqual(1, len(tracking_nodes))
        self.assertEqual("$(arg open_tracking_controller)", tracking_nodes[0].attrib.get("if"))

    def test_launch_can_show_detector_range_marker(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("./arg")}

        self.assertEqual("false", args["show_detector_range"])
        self.assertEqual("/onboard_detector/detector_range_marker", args["detector_range_marker_topic"])
        self.assertEqual("true", args["enforce_click_goal_range"])
        self.assertEqual("true", args["show_click_goal_range"])
        self.assertEqual("10.0", args["click_goal_range_x"])
        self.assertEqual("10.0", args["click_goal_range_y"])
        self.assertEqual("/autonomous_flight/click_goal_range_marker", args["click_goal_range_marker_topic"])

        marker_nodes = [
            node for node in root.findall("./node")
            if node.attrib.get("pkg") == "autonomous_flight"
            and node.attrib.get("type") == "publish_detector_range_marker.py"
        ]

        self.assertEqual(1, len(marker_nodes))
        marker_node = marker_nodes[0]
        self.assertEqual("publish_detector_range_marker", marker_node.attrib.get("name"))
        self.assertEqual("$(arg show_detector_range)", marker_node.attrib.get("if"))

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in marker_node.findall("./param")
        }
        self.assertEqual("$(arg odom_topic)", params["odom_topic"])
        self.assertEqual("$(arg detector_range_marker_topic)", params["marker_topic"])
        self.assertEqual("$(arg detector_range_x)", params["range_x"])
        self.assertEqual("$(arg detector_range_y)", params["range_y"])
        self.assertEqual("$(arg detector_range_z)", params["range_z"])

        click_marker_nodes = [
            node for node in root.findall("./node")
            if node.attrib.get("pkg") == "autonomous_flight"
            and node.attrib.get("type") == "publish_click_goal_range_marker.py"
        ]

        self.assertEqual(1, len(click_marker_nodes))
        click_marker_node = click_marker_nodes[0]
        self.assertEqual("publish_click_goal_range_marker", click_marker_node.attrib.get("name"))
        self.assertEqual("$(arg show_click_goal_range)", click_marker_node.attrib.get("if"))

        click_params = {
            param.attrib["name"]: param.attrib["value"]
            for param in click_marker_node.findall("./param")
        }
        self.assertEqual("$(arg odom_topic)", click_params["odom_topic"])
        self.assertEqual("$(arg click_goal_range_marker_topic)", click_params["marker_topic"])
        self.assertEqual("$(arg click_goal_range_x)", click_params["range_x"])
        self.assertEqual("$(arg click_goal_range_y)", click_params["range_y"])
        self.assertEqual("$(arg takeoff_height)", click_params["height"])

    def test_launch_enforces_click_goal_range(self):
        launch_path = PACKAGE_DIR / "launch" / "dynamic_navigation_lidar.launch"
        root = ET.parse(launch_path).getroot()

        params = {
            param.attrib["name"]: param.attrib["value"]
            for param in root.findall("./param")
        }

        self.assertEqual("$(arg enforce_click_goal_range)", params["/autonomous_flight/enforce_click_goal_range"])
        self.assertEqual("$(arg click_goal_range_x)", params["/autonomous_flight/click_goal_range_x"])
        self.assertEqual("$(arg click_goal_range_y)", params["/autonomous_flight/click_goal_range_y"])

    def test_navigation_lidar_detector_accepts_any_dynamic_box_under_three_meters(self):
        params = yaml.safe_load(
            (PACKAGE_DIR / "cfg" / "dynamic_navigation" / "dynamic_detector_lidar_param.yaml").read_text()
        )

        self.assertFalse(params["constrain_size"])
        self.assertEqual([3.0, 3.0, 3.0], params["max_dynamic_bbox_size"])

    def test_navigation_lidar_detector_uses_far_range_dynamic_profile(self):
        params = yaml.safe_load(
            (PACKAGE_DIR / "cfg" / "dynamic_navigation" / "dynamic_detector_lidar_param.yaml").read_text()
        )

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
