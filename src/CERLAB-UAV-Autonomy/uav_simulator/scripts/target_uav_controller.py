#!/usr/bin/env python3

import math
import re
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class TrajectorySample:
    position: Point3
    velocity: Point3
    yaw: float


@dataclass(frozen=True)
class Segment:
    start: Point3
    end: Point3
    length: float
    direction: Point3


def parse_waypoints(raw_waypoints: str) -> List[Point3]:
    waypoints = []
    for item in raw_waypoints.split(";"):
        item = item.strip()
        if not item:
            continue

        values = [token for token in re.split(r"[,\s]+", item) if token]
        if len(values) != 3:
            raise ValueError("Each target UAV waypoint must contain exactly x, y, z.")

        waypoints.append((float(values[0]), float(values[1]), float(values[2])))

    if len(waypoints) < 2:
        raise ValueError("Target UAV trajectory needs at least two waypoints.")

    return waypoints


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class WaypointPath:
    def __init__(self, waypoints: Sequence[Point3], speed: float, loop: bool):
        if len(waypoints) < 2:
            raise ValueError("WaypointPath needs at least two waypoints.")
        if speed <= 0.0:
            raise ValueError("Target UAV speed must be positive.")

        self.waypoints = list(waypoints)
        self.speed = float(speed)
        self.loop = bool(loop)
        self.segments = self._build_segments(self.waypoints, self.loop)
        if not self.segments:
            raise ValueError("Target UAV trajectory must contain at least one non-zero-length segment.")

        self.total_length = sum(segment.length for segment in self.segments)
        self.duration = self.total_length / self.speed

    def sample(self, elapsed_time: float) -> TrajectorySample:
        traveled = max(0.0, elapsed_time) * self.speed

        if self.loop:
            traveled = traveled % self.total_length
        elif traveled >= self.total_length:
            return TrajectorySample(position=self.segments[-1].end, velocity=(0.0, 0.0, 0.0), yaw=0.0)

        distance_cursor = 0.0
        for segment in self.segments:
            next_cursor = distance_cursor + segment.length
            if traveled <= next_cursor:
                ratio = (traveled - distance_cursor) / segment.length
                position = (
                    segment.start[0] + (segment.end[0] - segment.start[0]) * ratio,
                    segment.start[1] + (segment.end[1] - segment.start[1]) * ratio,
                    segment.start[2] + (segment.end[2] - segment.start[2]) * ratio,
                )
                velocity = (
                    segment.direction[0] * self.speed,
                    segment.direction[1] * self.speed,
                    segment.direction[2] * self.speed,
                )
                yaw = math.atan2(velocity[1], velocity[0])
                return TrajectorySample(position=position, velocity=velocity, yaw=yaw)

            distance_cursor = next_cursor

        last_segment = self.segments[-1]
        velocity = tuple(component * self.speed for component in last_segment.direction)
        return TrajectorySample(
            position=last_segment.end,
            velocity=velocity,
            yaw=math.atan2(velocity[1], velocity[0]),
        )

    @staticmethod
    def _build_segments(waypoints: Sequence[Point3], loop: bool) -> List[Segment]:
        points = list(waypoints)
        if loop:
            points = points + [points[0]]

        segments = []
        for start, end in zip(points, points[1:]):
            delta = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
            length = math.sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2])
            if length < 1e-6:
                continue
            direction = (delta[0] / length, delta[1] / length, delta[2] / length)
            segments.append(Segment(start=start, end=end, length=length, direction=direction))

        return segments


class TargetUavRosNode:
    def __init__(self):
        import rospy
        from gazebo_msgs.srv import SetModelState

        self.rospy = rospy
        self.model_name = rospy.get_param("~model_name", "tracking_target_uav")
        self.frame_id = rospy.get_param("~frame_id", "world")
        waypoints = parse_waypoints(rospy.get_param("~waypoints"))
        self.path = WaypointPath(
            waypoints=waypoints,
            speed=float(rospy.get_param("~speed", 1.2)),
            loop=bool(rospy.get_param("~loop", True)),
        )

        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.prediction_horizon = float(rospy.get_param("~prediction_horizon", 8.0))
        self.prediction_dt = float(rospy.get_param("~prediction_dt", 0.5))
        self.path_publish_period = float(rospy.get_param("~path_publish_period", 1.0))

        self.odom_pub = rospy.Publisher(rospy.get_param("~odom_topic", "/tracking_target/odom"), self._odom_type(), queue_size=10)
        self.path_pub = rospy.Publisher(rospy.get_param("~path_topic", "/tracking_target/path"), self._path_type(), queue_size=1, latch=True)
        self.prediction_pub = rospy.Publisher(
            rospy.get_param("~prediction_topic", "/tracking_target/prediction"),
            self._path_type(),
            queue_size=1,
        )

        service_name = rospy.get_param("~set_model_state_service", "/gazebo/set_model_state")
        rospy.wait_for_service(service_name)
        self.set_model_state = rospy.ServiceProxy(service_name, SetModelState)
        self.start_time = rospy.Time.now()
        self.last_path_publish_time = rospy.Time(0)

    def spin(self):
        rate = self.rospy.Rate(self.rate_hz)
        while not self.rospy.is_shutdown():
            now = self.rospy.Time.now()
            elapsed_time = (now - self.start_time).to_sec()
            sample = self.path.sample(elapsed_time)
            self._set_gazebo_model(sample)
            self._publish_odom(now, sample)
            self._publish_prediction(now, elapsed_time)

            if (now - self.last_path_publish_time).to_sec() >= self.path_publish_period:
                self._publish_nominal_path(now)
                self.last_path_publish_time = now

            rate.sleep()

    def _set_gazebo_model(self, sample: TrajectorySample):
        from gazebo_msgs.msg import ModelState

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = self.frame_id
        self._fill_pose(state.pose, sample)
        state.twist.linear.x = sample.velocity[0]
        state.twist.linear.y = sample.velocity[1]
        state.twist.linear.z = sample.velocity[2]
        try:
            self.set_model_state(state)
        except Exception as exc:
            self.rospy.logwarn_throttle(2.0, "Failed to set target UAV model state: %s", exc)

    def _publish_odom(self, stamp, sample: TrajectorySample):
        from nav_msgs.msg import Odometry

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.model_name + "/base_link"
        self._fill_pose(odom.pose.pose, sample)
        odom.twist.twist.linear.x = sample.velocity[0]
        odom.twist.twist.linear.y = sample.velocity[1]
        odom.twist.twist.linear.z = sample.velocity[2]
        self.odom_pub.publish(odom)

    def _publish_nominal_path(self, stamp):
        path_msg = self._make_path_message(stamp)
        for waypoint in self.path.waypoints:
            sample = TrajectorySample(position=waypoint, velocity=(0.0, 0.0, 0.0), yaw=0.0)
            path_msg.poses.append(self._make_pose_stamped(stamp, sample))
        if self.path.loop and self.path.waypoints[-1] != self.path.waypoints[0]:
            sample = TrajectorySample(position=self.path.waypoints[0], velocity=(0.0, 0.0, 0.0), yaw=0.0)
            path_msg.poses.append(self._make_pose_stamped(stamp, sample))
        self.path_pub.publish(path_msg)

    def _publish_prediction(self, stamp, elapsed_time: float):
        path_msg = self._make_path_message(stamp)
        count = max(1, int(math.ceil(self.prediction_horizon / self.prediction_dt)))
        for index in range(count + 1):
            sample = self.path.sample(elapsed_time + index * self.prediction_dt)
            path_msg.poses.append(self._make_pose_stamped(stamp, sample))
        self.prediction_pub.publish(path_msg)

    def _make_pose_stamped(self, stamp, sample: TrajectorySample):
        from geometry_msgs.msg import PoseStamped

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = self.frame_id
        self._fill_pose(pose_stamped.pose, sample)
        return pose_stamped

    def _make_path_message(self, stamp):
        from nav_msgs.msg import Path

        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id
        return path_msg

    @staticmethod
    def _fill_pose(pose, sample: TrajectorySample):
        qx, qy, qz, qw = yaw_to_quaternion(sample.yaw)
        pose.position.x = sample.position[0]
        pose.position.y = sample.position[1]
        pose.position.z = sample.position[2]
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


def main():
    import rospy

    rospy.init_node("target_uav_controller")
    node = TargetUavRosNode()
    node.spin()


if __name__ == "__main__":
    main()
