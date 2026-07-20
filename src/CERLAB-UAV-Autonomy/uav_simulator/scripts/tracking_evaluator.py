#!/usr/bin/env python3

import csv
import math
from pathlib import Path
import sys
from typing import Dict, Iterable, Optional, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from scenario_manager import load_scenario_config, project_target_position


Point3 = Tuple[float, float, float]


SAMPLE_FIELDS = [
    "run_id",
    "time",
    "scenario_id",
    "uav_x",
    "uav_y",
    "uav_z",
    "target_x",
    "target_y",
    "target_z",
    "target_track_x",
    "target_track_y",
    "target_track_z",
    "target_vx",
    "target_vy",
    "target_vz",
    "tracking_distance",
    "tracking_error",
    "relative_speed",
    "target_distance_violation",
    "relative_speed_violation",
    "min_dynamic_obstacle_distance",
    "min_static_obstacle_distance",
    "min_target_distance",
    "collision_dynamic",
    "collision_static",
    "collision_target",
    "planner_replan_id",
    "planner_success",
    "planner_mode",
    "total_time_ms",
    "search_time_ms",
    "optimize_time_ms",
    "st_expand_time_ms",
    "st_time_layers",
    "st_nodes_expanded",
    "st_edges_checked",
    "checked_obstacles",
    "relevant_obstacles",
    "pruned_collision",
    "pruned_dominated",
    "pruned_recovery",
    "pruned_total",
    "replan_count",
    "min_st_distance",
    "planner_tracking_distance_error",
    "planner_relative_velocity_error",
    "recovery_time_est",
    "trajectory_length",
]


EVENT_FIELDS = ["run_id", "time", "scenario_id", "event_type", "object_id", "value", "details"]


def _distance(a: Point3, b: Point3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _vector_norm(vector: Point3) -> float:
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


def _nan() -> float:
    return float("nan")


def tracking_metrics(
    uav_position: Point3,
    uav_velocity: Point3,
    target_position: Point3,
    target_velocity: Point3,
    z_track: float,
    desired_distance: float,
    min_distance: float,
    max_distance: float,
    max_relative_speed: float,
) -> Dict:
    target_track = project_target_position(target_position, z_track)
    tracking_distance = _distance(uav_position, target_track)
    relative_velocity = (
        uav_velocity[0] - target_velocity[0],
        uav_velocity[1] - target_velocity[1],
        uav_velocity[2] - target_velocity[2],
    )
    relative_speed = _vector_norm(relative_velocity)
    return {
        "target_track": target_track,
        "tracking_distance": tracking_distance,
        "tracking_error": abs(tracking_distance - desired_distance),
        "relative_speed": relative_speed,
        "target_distance_violation": tracking_distance < min_distance or tracking_distance > max_distance,
        "relative_speed_violation": relative_speed > max_relative_speed,
    }


def uav_world_position_from_mavros(mavros_local_position: Point3, uav_initial_pose) -> Point3:
    return (
        float(uav_initial_pose[0]) + mavros_local_position[0],
        float(uav_initial_pose[1]) + mavros_local_position[1],
        float(uav_initial_pose[2]) + mavros_local_position[2],
    )


def dynamic_obstacle_clearance(
    uav_position: Point3,
    obstacle_position: Point3,
    uav_radius: float,
    obstacle_radius: float,
) -> float:
    return _distance(uav_position, obstacle_position) - uav_radius - obstacle_radius


def target_clearance(
    uav_position: Point3,
    target_position: Point3,
    z_track: float,
    uav_radius: float,
    target_radius: float,
) -> float:
    return _distance(uav_position, project_target_position(target_position, z_track)) - uav_radius - target_radius


def static_obstacle_clearance(uav_position: Point3, obstacle: dict, uav_radius: float) -> float:
    if obstacle["type"] == "cylinder":
        base = obstacle["position"]
        height = float(obstacle["height"])
        radius = float(obstacle["radius"])
        if uav_position[2] < float(base[2]) or uav_position[2] > float(base[2]) + height:
            vertical = min(abs(uav_position[2] - float(base[2])), abs(uav_position[2] - (float(base[2]) + height)))
        else:
            vertical = 0.0
        xy_clearance = math.hypot(uav_position[0] - float(base[0]), uav_position[1] - float(base[1])) - radius - uav_radius
        if vertical <= 1e-9:
            return xy_clearance
        return math.sqrt(max(0.0, xy_clearance) ** 2 + vertical ** 2)

    if obstacle["type"] in ("box", "wall"):
        center = obstacle["center"]
        size = obstacle["size"]
        dx = max(abs(uav_position[0] - float(center[0])) - 0.5 * float(size[0]), 0.0)
        dy = max(abs(uav_position[1] - float(center[1])) - 0.5 * float(size[1]), 0.0)
        dz = max(abs(uav_position[2] - float(center[2])) - 0.5 * float(size[2]), 0.0)
        return math.sqrt(dx * dx + dy * dy + dz * dz) - uav_radius

    raise ValueError("Unsupported static obstacle type: {}".format(obstacle["type"]))


def planner_missing_defaults() -> Dict:
    return {
        "planner_replan_id": -1,
        "planner_success": False,
        "planner_mode": "missing",
        "total_time_ms": _nan(),
        "search_time_ms": _nan(),
        "optimize_time_ms": _nan(),
        "st_expand_time_ms": _nan(),
        "st_time_layers": -1,
        "st_nodes_expanded": -1,
        "st_edges_checked": -1,
        "checked_obstacles": -1,
        "relevant_obstacles": -1,
        "pruned_collision": -1,
        "pruned_dominated": -1,
        "pruned_recovery": -1,
        "pruned_total": -1,
        "replan_count": -1,
        "min_st_distance": _nan(),
        "planner_tracking_distance_error": _nan(),
        "planner_relative_velocity_error": _nan(),
        "recovery_time_est": _nan(),
    }


def optional_message_type(importer):
    try:
        return importer()
    except ImportError:
        return None


def planner_diag_to_dict(planner_diag) -> Dict:
    if planner_diag is None:
        return planner_missing_defaults()
    return {
        "planner_replan_id": planner_diag.replan_id,
        "planner_success": planner_diag.success,
        "planner_mode": planner_diag.planner_mode,
        "total_time_ms": planner_diag.total_time_ms,
        "search_time_ms": planner_diag.search_time_ms,
        "optimize_time_ms": planner_diag.optimize_time_ms,
        "st_expand_time_ms": planner_diag.st_expand_time_ms,
        "st_time_layers": planner_diag.st_time_layers,
        "st_nodes_expanded": planner_diag.st_nodes_expanded,
        "st_edges_checked": planner_diag.st_edges_checked,
        "checked_obstacles": planner_diag.checked_obstacles,
        "relevant_obstacles": planner_diag.relevant_obstacles,
        "pruned_collision": planner_diag.pruned_collision,
        "pruned_dominated": planner_diag.pruned_dominated,
        "pruned_recovery": planner_diag.pruned_recovery,
        "pruned_total": planner_diag.pruned_total,
        "replan_count": planner_diag.replan_count,
        "min_st_distance": planner_diag.min_st_distance,
        "planner_tracking_distance_error": planner_diag.tracking_distance_error,
        "planner_relative_velocity_error": planner_diag.relative_velocity_error,
        "recovery_time_est": planner_diag.recovery_time_est,
    }


def _min_or_nan(values: Iterable[float]) -> float:
    values = list(values)
    return min(values) if values else _nan()


def build_sample_row(
    run_id: str,
    scenario_id: str,
    time_sec: float,
    uav_position: Point3,
    uav_velocity: Point3,
    target_position: Point3,
    target_velocity: Point3,
    z_track: float,
    tracking_config: dict,
    evaluation_config: dict,
    dynamic_obstacles: Iterable[dict],
    static_obstacles: Iterable[dict],
    planner_diag,
    trajectory_length: float,
) -> Dict:
    metrics = tracking_metrics(
        uav_position,
        uav_velocity,
        target_position,
        target_velocity,
        z_track,
        float(tracking_config["desired_distance"]),
        float(tracking_config["min_distance"]),
        float(tracking_config["max_distance"]),
        float(tracking_config["max_relative_speed"]),
    )
    uav_radius = float(evaluation_config["uav_radius"])
    target_radius = float(evaluation_config["target_radius"])
    collision_margin = float(evaluation_config["collision_margin"])

    dynamic_clearances = [
        dynamic_obstacle_clearance(uav_position, obstacle["position"], uav_radius, float(obstacle["size"]))
        for obstacle in dynamic_obstacles
    ]
    static_clearances = [
        static_obstacle_clearance(uav_position, obstacle, uav_radius)
        for obstacle in static_obstacles
    ]
    min_target_clearance = target_clearance(uav_position, target_position, z_track, uav_radius, target_radius)
    planner = planner_diag_to_dict(planner_diag)

    row = {
        "run_id": run_id,
        "time": time_sec,
        "scenario_id": scenario_id,
        "uav_x": uav_position[0],
        "uav_y": uav_position[1],
        "uav_z": uav_position[2],
        "target_x": target_position[0],
        "target_y": target_position[1],
        "target_z": target_position[2],
        "target_track_x": metrics["target_track"][0],
        "target_track_y": metrics["target_track"][1],
        "target_track_z": metrics["target_track"][2],
        "target_vx": target_velocity[0],
        "target_vy": target_velocity[1],
        "target_vz": target_velocity[2],
        "tracking_distance": metrics["tracking_distance"],
        "tracking_error": metrics["tracking_error"],
        "relative_speed": metrics["relative_speed"],
        "target_distance_violation": metrics["target_distance_violation"],
        "relative_speed_violation": metrics["relative_speed_violation"],
        "min_dynamic_obstacle_distance": _min_or_nan(dynamic_clearances),
        "min_static_obstacle_distance": _min_or_nan(static_clearances),
        "min_target_distance": min_target_clearance,
        "collision_dynamic": any(clearance < collision_margin for clearance in dynamic_clearances),
        "collision_static": any(clearance < collision_margin for clearance in static_clearances),
        "collision_target": min_target_clearance < collision_margin,
        "trajectory_length": trajectory_length,
    }
    row.update(planner)
    return row


def path_length(path_msg) -> float:
    poses = path_msg.poses
    total = 0.0
    for start, end in zip(poses, poses[1:]):
        a = start.pose.position
        b = end.pose.position
        total += _distance((a.x, a.y, a.z), (b.x, b.y, b.z))
    return total


class TrackingEvaluatorNode:
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry, Path as RosPath
        from uav_simulator.msg import DynamicObstacleArray

        self.rospy = rospy
        self.scenario = rospy.get_param("~scenario", "scenario1")
        self.scenario_dir = rospy.get_param(
            "~scenario_dir",
            str(Path(__file__).resolve().parents[1] / "config" / "tracking_scenarios"),
        )
        self.config = load_scenario_config(self.scenario, self.scenario_dir)
        self.run_id = self.config["scenario"].get("run_id", "default")
        self.scenario_id = self.config["scenario"]["id"]
        self.z_track = float(self.config["scenario"]["z_track"])
        self.uav_initial_pose = self.config["uav"]["initial_pose"]
        self.rate_hz = float(rospy.get_param("~rate", self.config["scenario"].get("update_rate", 30.0)))
        self.output_dir = Path(self.config["evaluation"]["output_dir"]) / self.run_id / self.scenario_id
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.uav_odom = None
        self.target_odom = None
        self.dynamic_obstacles = []
        self.planner_diag = None
        self.trajectory_length = _nan()

        rospy.Subscriber(rospy.get_param("~uav_odom_topic", "/mavros/local_position/odom"), Odometry, self._on_uav_odom)
        rospy.Subscriber(rospy.get_param("~target_odom_topic", "/target/odom"), Odometry, self._on_target_odom)
        rospy.Subscriber(rospy.get_param("~dynamic_obstacles_topic", "/dynamic_obstacles"), DynamicObstacleArray, self._on_dynamic_obstacles)
        planner_diag_type = optional_message_type(self._planner_eval_diagnostics_type)
        if planner_diag_type is None:
            rospy.logwarn("PlannerEvalDiagnostics message is not generated yet; planner diagnostics will be recorded as missing.")
        else:
            rospy.Subscriber(rospy.get_param("~planner_diagnostics_topic", "/planner/eval_diagnostics"), planner_diag_type, self._on_planner_diag)
        rospy.Subscriber(rospy.get_param("~planning_trajectory_topic", "/planning/trajectory"), RosPath, self._on_planning_trajectory)

        self.samples_file = (self.output_dir / "samples.csv").open("w", newline="")
        self.events_file = (self.output_dir / "events.csv").open("w", newline="")
        self.samples_writer = csv.DictWriter(self.samples_file, fieldnames=SAMPLE_FIELDS)
        self.events_writer = csv.DictWriter(self.events_file, fieldnames=EVENT_FIELDS)
        self.samples_writer.writeheader()
        self.events_writer.writeheader()
        self.start_time = rospy.Time.now()

    def spin(self):
        rate = self.rospy.Rate(self.rate_hz)
        try:
            while not self.rospy.is_shutdown():
                if self.uav_odom is not None and self.target_odom is not None:
                    self._write_sample()
                rate.sleep()
        except self.rospy.exceptions.ROSInterruptException:
            pass
        finally:
            self.samples_file.close()
            self.events_file.close()

    def _on_uav_odom(self, msg):
        self.uav_odom = msg

    def _on_target_odom(self, msg):
        self.target_odom = msg

    def _on_dynamic_obstacles(self, msg):
        obstacles = []
        for obstacle in msg.obstacles:
            obstacles.append(
                {
                    "name": obstacle.name,
                    "size": obstacle.size,
                    "height": obstacle.height,
                    "position": (
                        obstacle.pose.position.x,
                        obstacle.pose.position.y,
                        obstacle.pose.position.z,
                    ),
                }
            )
        self.dynamic_obstacles = obstacles

    def _on_planner_diag(self, msg):
        self.planner_diag = msg

    def _on_planning_trajectory(self, msg):
        self.trajectory_length = path_length(msg)

    def _write_sample(self):
        now = self.rospy.Time.now()
        row = build_sample_row(
            run_id=self.run_id,
            scenario_id=self.scenario_id,
            time_sec=(now - self.start_time).to_sec(),
            uav_position=uav_world_position_from_mavros(self._position(self.uav_odom), self.uav_initial_pose),
            uav_velocity=self._velocity(self.uav_odom),
            target_position=self._position(self.target_odom),
            target_velocity=self._velocity(self.target_odom),
            z_track=self.z_track,
            tracking_config=self.config["evaluation"]["tracking"],
            evaluation_config=self.config["evaluation"],
            dynamic_obstacles=self.dynamic_obstacles,
            static_obstacles=self.config.get("static_obstacles_eval", []),
            planner_diag=self.planner_diag,
            trajectory_length=self.trajectory_length,
        )
        self.samples_writer.writerow(row)
        self.samples_file.flush()

    @staticmethod
    def _position(odom) -> Point3:
        p = odom.pose.pose.position
        return (p.x, p.y, p.z)

    @staticmethod
    def _velocity(odom) -> Point3:
        v = odom.twist.twist.linear
        return (v.x, v.y, v.z)

    @staticmethod
    def _planner_eval_diagnostics_type():
        from uav_simulator.msg import PlannerEvalDiagnostics

        return PlannerEvalDiagnostics


def main():
    import rospy

    rospy.init_node("tracking_evaluator")
    TrackingEvaluatorNode().spin()


if __name__ == "__main__":
    main()
