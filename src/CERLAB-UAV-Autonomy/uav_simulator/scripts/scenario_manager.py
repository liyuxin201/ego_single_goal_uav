#!/usr/bin/env python3

import math
from pathlib import Path
import sys
from typing import Dict, Iterable, Tuple

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from tracking_trajectory import Trajectory, TrajectorySample


Point3 = Tuple[float, float, float]


SCENARIO_FILES = {
    "scenario1": "scenario1_normal_tracking.yaml",
    "scenario2": "scenario2_adaptive_stspace.yaml",
    "scenario3": "scenario3_pruning_search.yaml",
    "scenario4a": "scenario4a_lateral_crossing.yaml",
    "scenario4b": "scenario4b_head_on.yaml",
    "scenario4c": "scenario4c_same_direction_slow.yaml",
    "scenario5": "scenario5_full_complex.yaml",
}


def resolve_scenario_path(scenario: str, scenario_dir) -> Path:
    scenario_dir = Path(scenario_dir)
    candidate = Path(scenario)
    if candidate.is_absolute() or candidate.suffix in (".yaml", ".yml"):
        return candidate if candidate.is_absolute() else scenario_dir / candidate
    if scenario not in SCENARIO_FILES:
        raise ValueError("Unknown tracking scenario: {}".format(scenario))
    return scenario_dir / SCENARIO_FILES[scenario]


def load_scenario_config(scenario: str, scenario_dir) -> dict:
    path = resolve_scenario_path(scenario, scenario_dir)
    with path.open("r") as handle:
        return yaml.safe_load(handle)


def project_target_position(position: Point3, z_track: float) -> Point3:
    return (float(position[0]), float(position[1]), float(z_track))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def yaw_to_quaternion(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def cylinder_sdf(name: str, radius: float, height: float, static: bool) -> str:
    static_text = "true" if static else "false"
    gravity_text = "false"
    return """<sdf version="1.6">
  <model name="{name}">
    <static>{static_text}</static>
    <link name="link">
      <gravity>{gravity_text}</gravity>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.25 0.1 1</ambient>
          <diffuse>0.8 0.25 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""".format(
        name=name,
        static_text=static_text,
        gravity_text=gravity_text,
        radius=radius,
        height=height,
    )


def build_obstacle_record(config: dict, sample: TrajectorySample) -> Dict:
    height = float(config.get("height", 1.8))
    return {
        "name": config["name"],
        "obstacle_type": config.get("model", "cylinder"),
        "size": float(config["radius"]),
        "height": height,
        "is_dynamic": True,
        "position": (sample.position[0], sample.position[1], sample.position[2] + 0.5 * height),
        "velocity": sample.velocity,
        "yaw": sample.yaw,
    }


def motion_elapsed(now_sec: float, motion_start_sec) -> float:
    if motion_start_sec is None:
        return 0.0
    return max(0.0, float(now_sec) - float(motion_start_sec))


def sample_with_start_delay(trajectory: Trajectory, elapsed_time: float, start_delay: float = 0.0) -> TrajectorySample:
    delay = max(0.0, float(start_delay))
    if float(elapsed_time) <= delay:
        sample = trajectory.sample(0.0)
        return TrajectorySample(position=sample.position, velocity=(0.0, 0.0, 0.0), yaw=sample.yaw)
    return trajectory.sample(float(elapsed_time) - delay)


def motion_start_allowed(uav_ready: bool, wait_for_motion_start: bool, motion_start_requested: bool) -> bool:
    return bool(uav_ready) and (not bool(wait_for_motion_start) or bool(motion_start_requested))


def uav_ready_from_inputs(
    wait_for_uav_ready: bool,
    uav_odom,
    mavros_state,
    ready_min_z: float,
    require_armed: bool,
    require_offboard: bool,
) -> bool:
    if not wait_for_uav_ready:
        return True
    if uav_odom is None:
        return False
    if float(uav_odom.pose.pose.position.z) < float(ready_min_z):
        return False
    if require_armed or require_offboard:
        if mavros_state is None:
            return False
        if require_armed and not bool(mavros_state.armed):
            return False
        if require_offboard and str(mavros_state.mode).upper() != "OFFBOARD":
            return False
    return True


def static_obstacle_spawn_pose(config: dict) -> Point3:
    if config["type"] == "cylinder":
        position = config["position"]
        return (float(position[0]), float(position[1]), float(position[2]) + 0.5 * float(config["height"]))
    if config["type"] == "box":
        center = config["center"]
        return (float(center[0]), float(center[1]), float(center[2]))
    raise ValueError("Unsupported static obstacle type: {}".format(config["type"]))


def box_sdf(name: str, size: Iterable[float], static: bool = True) -> str:
    size_text = "{} {} {}".format(float(size[0]), float(size[1]), float(size[2]))
    static_text = "true" if static else "false"
    return """<sdf version="1.6">
  <model name="{name}">
    <static>{static_text}</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>{size_text}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{size_text}</size></box>
        </geometry>
        <material>
          <ambient>0.45 0.45 0.45 1</ambient>
          <diffuse>0.45 0.45 0.45 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""".format(name=name, static_text=static_text, size_text=size_text)


class ScenarioManagerNode:
    def __init__(self):
        import rospy
        from gazebo_msgs.srv import SetModelState, SpawnModel
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool

        self.rospy = rospy
        self.scenario = rospy.get_param("~scenario", "scenario1")
        self.scenario_dir = rospy.get_param(
            "~scenario_dir",
            str(Path(__file__).resolve().parents[1] / "config" / "tracking_scenarios"),
        )
        self.config = load_scenario_config(self.scenario, self.scenario_dir)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.z_track = float(self.config["scenario"]["z_track"])
        self.rate_hz = float(self.config["scenario"].get("update_rate", 30.0))
        self.prediction_horizon = float(rospy.get_param("~prediction_horizon", 3.0))
        self.prediction_dt = float(rospy.get_param("~prediction_dt", 0.2))
        self.path_dt = float(rospy.get_param("~path_dt", 0.5))
        self.wait_for_uav_ready = as_bool(rospy.get_param("~wait_for_uav_ready", True))
        self.uav_ready_min_z = float(rospy.get_param("~uav_ready_min_z", 0.8))
        self.uav_ready_start_delay = float(rospy.get_param("~uav_ready_start_delay", 1.0))
        self.uav_ready_require_armed = as_bool(rospy.get_param("~uav_ready_require_armed", True))
        self.uav_ready_require_offboard = as_bool(rospy.get_param("~uav_ready_require_offboard", True))
        self.wait_for_motion_start = as_bool(rospy.get_param("~wait_for_motion_start", True))

        self.target_trajectory = Trajectory.from_config(self.config["target"]["trajectory"])
        self.obstacle_trajectories = {
            obstacle["name"]: Trajectory.from_config(obstacle["trajectory"])
            for obstacle in self.config.get("dynamic_obstacles", [])
        }

        self.target_odom_pub = rospy.Publisher(rospy.get_param("~target_odom_topic", "/target/odom"), self._odom_type(), queue_size=10)
        self.target_path_pub = rospy.Publisher(rospy.get_param("~target_path_topic", "/target/path"), self._path_type(), queue_size=1, latch=True)
        self.target_prediction_pub = rospy.Publisher(
            rospy.get_param("~target_prediction_topic", "/target/prediction"),
            self._path_type(),
            queue_size=10,
        )
        self.dynamic_obstacles_pub = rospy.Publisher(
            rospy.get_param("~dynamic_obstacles_topic", "/dynamic_obstacles"),
            self._dynamic_obstacle_array_type(),
            queue_size=10,
        )

        self.uav_odom = None
        self.mavros_state = None
        self.motion_start_requested = not self.wait_for_motion_start
        rospy.Subscriber(rospy.get_param("~uav_odom_topic", "/mavros/local_position/odom"), Odometry, self._on_uav_odom)
        rospy.Subscriber(rospy.get_param("~motion_start_topic", "/tracking_scenario/start_motion"), Bool, self._on_motion_start)
        try:
            from mavros_msgs.msg import State

            rospy.Subscriber(rospy.get_param("~mavros_state_topic", "/mavros/state"), State, self._on_mavros_state)
        except ImportError:
            if self.uav_ready_require_armed or self.uav_ready_require_offboard:
                rospy.logwarn("mavros_msgs is unavailable; UAV ready state cannot use armed/offboard checks.")

        spawn_service = rospy.get_param("~spawn_model_service", "/gazebo/spawn_sdf_model")
        set_state_service = rospy.get_param("~set_model_state_service", "/gazebo/set_model_state")
        rospy.wait_for_service(spawn_service)
        rospy.wait_for_service(set_state_service)
        self.spawn_model = rospy.ServiceProxy(spawn_service, SpawnModel)
        self.set_model_state = rospy.ServiceProxy(set_state_service, SetModelState)
        self.start_time = rospy.Time.now()
        self.motion_start_time = None if self.wait_for_uav_ready else self.start_time
        self.spawned_names = set()

    def spin(self):
        self._apply_initial_uav_pose()
        self._spawn_configured_models()
        self._publish_target_path(self.rospy.Time.now())

        rate = self.rospy.Rate(self.rate_hz)
        try:
            while not self.rospy.is_shutdown():
                now = self.rospy.Time.now()
                elapsed = self._motion_elapsed(now)
                target_sample = self.target_trajectory.sample(elapsed)
                if elapsed <= 1e-9:
                    target_sample = self._stopped_sample(target_sample)
                self._set_model_state(
                    self.config["target"]["model_name"],
                    (target_sample.position[0], target_sample.position[1], 0.5 * float(self.config["target"].get("height", 0.4))),
                    target_sample,
                )
                self._publish_target_odom(now, target_sample)
                self._publish_target_prediction(now, elapsed)
                self._publish_dynamic_obstacles(now, elapsed)
                rate.sleep()
        except self.rospy.exceptions.ROSInterruptException:
            pass

    def _motion_elapsed(self, now):
        if self.motion_start_time is None and self._motion_can_start():
            self.motion_start_time = now + self.rospy.Duration.from_sec(max(0.0, self.uav_ready_start_delay))
            self.rospy.loginfo(
                "UAV ready detected; tracking scenario motion starts after %.2f s.",
                max(0.0, self.uav_ready_start_delay),
            )
        return motion_elapsed(now.to_sec(), None if self.motion_start_time is None else self.motion_start_time.to_sec())

    def _motion_can_start(self) -> bool:
        return motion_start_allowed(self._uav_is_ready(), self.wait_for_motion_start, self.motion_start_requested)

    def _uav_is_ready(self) -> bool:
        return uav_ready_from_inputs(
            self.wait_for_uav_ready,
            self.uav_odom,
            self.mavros_state,
            self.uav_ready_min_z,
            self.uav_ready_require_armed,
            self.uav_ready_require_offboard,
        )

    @staticmethod
    def _stopped_sample(sample: TrajectorySample) -> TrajectorySample:
        return TrajectorySample(position=sample.position, velocity=(0.0, 0.0, 0.0), yaw=sample.yaw)

    def _spawn_configured_models(self):
        target = self.config["target"]
        self._spawn_cylinder(
            target["model_name"],
            float(target.get("radius", 0.35)),
            float(target.get("height", 0.4)),
            self.target_trajectory.sample(0.0).position,
            static=False,
        )

        for obstacle in self.config.get("dynamic_obstacles", []):
            trajectory = self.obstacle_trajectories[obstacle["name"]]
            self._spawn_cylinder(
                obstacle["name"],
                float(obstacle["radius"]),
                float(obstacle["height"]),
                trajectory.sample(0.0).position,
                static=False,
            )

        for obstacle in self.config.get("static_obstacles_eval", []):
            if obstacle["type"] == "cylinder":
                self._spawn_cylinder(
                    obstacle["name"],
                    float(obstacle["radius"]),
                    float(obstacle["height"]),
                    static_obstacle_spawn_pose(obstacle),
                    static=True,
                    position_is_center=True,
                )
            elif obstacle["type"] == "box":
                self._spawn_model(
                    obstacle["name"],
                    box_sdf(obstacle["name"], obstacle["size"], static=True),
                    static_obstacle_spawn_pose(obstacle),
                )

    def _apply_initial_uav_pose(self):
        pose = self.config.get("uav", {}).get("initial_pose")
        model_name = self.config.get("uav", {}).get("model")
        if not pose or not model_name:
            return

        yaw = float(pose[3]) if len(pose) > 3 else 0.0
        sample = TrajectorySample(
            position=(float(pose[0]), float(pose[1]), float(pose[2])),
            velocity=(0.0, 0.0, 0.0),
            yaw=yaw,
        )
        attempts = int(self.rospy.get_param("~uav_initial_pose_attempts", 30))
        for _ in range(max(1, attempts)):
            if self._set_model_state(model_name, sample.position, sample):
                return
            self.rospy.sleep(0.1)
        self.rospy.logwarn("Failed to apply initial UAV pose for model %s", model_name)

    def _on_uav_odom(self, msg):
        self.uav_odom = msg

    def _on_mavros_state(self, msg):
        self.mavros_state = msg

    def _on_motion_start(self, msg):
        if msg.data and not self.motion_start_requested:
            self.rospy.loginfo("Received tracking scenario motion start signal.")
        self.motion_start_requested = self.motion_start_requested or bool(msg.data)

    def _spawn_cylinder(self, name: str, radius: float, height: float, ground_position: Point3, static: bool, position_is_center: bool = False):
        if position_is_center:
            spawn_position = ground_position
        else:
            spawn_position = (ground_position[0], ground_position[1], ground_position[2] + 0.5 * height)
        self._spawn_model(name, cylinder_sdf(name, radius, height, static), spawn_position)

    def _spawn_model(self, name: str, sdf: str, position: Point3):
        if name in self.spawned_names:
            return
        from geometry_msgs.msg import Pose

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        try:
            self.spawn_model(name, sdf, "", pose, self.frame_id)
            self.spawned_names.add(name)
        except Exception as exc:
            self.rospy.logwarn("Failed to spawn %s: %s", name, exc)

    def _set_model_state(self, model_name: str, position: Point3, sample: TrajectorySample):
        from gazebo_msgs.msg import ModelState

        state = ModelState()
        state.model_name = model_name
        state.reference_frame = self.frame_id
        state.pose.position.x = position[0]
        state.pose.position.y = position[1]
        state.pose.position.z = position[2]
        qx, qy, qz, qw = yaw_to_quaternion(sample.yaw)
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.twist.linear.x = sample.velocity[0]
        state.twist.linear.y = sample.velocity[1]
        state.twist.linear.z = sample.velocity[2]
        try:
            response = self.set_model_state(state)
            return bool(getattr(response, "success", True))
        except Exception as exc:
            self.rospy.logwarn_throttle(2.0, "Failed to set model state for %s: %s", model_name, exc)
            return False

    def _publish_target_odom(self, stamp, sample: TrajectorySample):
        odom = self._odom_type()()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.config["target"]["model_name"]
        self._fill_pose(odom.pose.pose, sample.position, sample.yaw)
        odom.twist.twist.linear.x = sample.velocity[0]
        odom.twist.twist.linear.y = sample.velocity[1]
        odom.twist.twist.linear.z = sample.velocity[2]
        self.target_odom_pub.publish(odom)

    def _publish_target_path(self, stamp):
        path = self._path_type()()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        for sample in self.target_trajectory.sample_path(self.path_dt):
            path.poses.append(self._pose_stamped(stamp, sample.position, sample.yaw))
        self.target_path_pub.publish(path)

    def _publish_target_prediction(self, stamp, elapsed: float):
        path = self._path_type()()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        count = max(1, int(math.ceil(self.prediction_horizon / self.prediction_dt)))
        for index in range(count + 1):
            sample = self.target_trajectory.sample(elapsed + index * self.prediction_dt)
            path.poses.append(self._pose_stamped(stamp, sample.position, sample.yaw))
        self.target_prediction_pub.publish(path)

    def _publish_dynamic_obstacles(self, stamp, elapsed: float):
        array = self._dynamic_obstacle_array_type()()
        array.header.stamp = stamp
        array.header.frame_id = self.frame_id
        for config in self.config.get("dynamic_obstacles", []):
            trajectory_config = config.get("trajectory", {})
            sample = sample_with_start_delay(
                self.obstacle_trajectories[config["name"]],
                elapsed,
                float(trajectory_config.get("start_delay", 0.0)),
            )
            record = build_obstacle_record(config, sample)
            self._set_model_state(config["name"], record["position"], sample)
            state = self._dynamic_obstacle_state_type()()
            state.header.stamp = stamp
            state.header.frame_id = self.frame_id
            state.name = record["name"]
            state.obstacle_type = record["obstacle_type"]
            state.size = record["size"]
            state.height = record["height"]
            state.is_dynamic = record["is_dynamic"]
            self._fill_pose(state.pose, record["position"], record["yaw"])
            state.twist.linear.x = record["velocity"][0]
            state.twist.linear.y = record["velocity"][1]
            state.twist.linear.z = record["velocity"][2]
            array.obstacles.append(state)
        self.dynamic_obstacles_pub.publish(array)

    def _pose_stamped(self, stamp, position: Point3, yaw: float):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        self._fill_pose(pose.pose, position, yaw)
        return pose

    @staticmethod
    def _fill_pose(pose, position: Point3, yaw: float):
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

    @staticmethod
    def _odom_type():
        from nav_msgs.msg import Odometry

        return Odometry

    @staticmethod
    def _path_type():
        from nav_msgs.msg import Path

        return Path

    @staticmethod
    def _dynamic_obstacle_array_type():
        from uav_simulator.msg import DynamicObstacleArray

        return DynamicObstacleArray

    @staticmethod
    def _dynamic_obstacle_state_type():
        from uav_simulator.msg import DynamicObstacleState

        return DynamicObstacleState


def main():
    import rospy

    rospy.init_node("scenario_manager")
    ScenarioManagerNode().spin()


if __name__ == "__main__":
    main()
