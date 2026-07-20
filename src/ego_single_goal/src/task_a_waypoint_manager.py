#!/usr/bin/env python3

import math
from dataclasses import dataclass

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
except ImportError:
    rospy = None
    PoseStamped = None
    Odometry = None
    Path = None

try:
    from mavros_msgs.msg import State
except ImportError:
    State = None


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


def distance(a, b):
    return math.sqrt(
        (a.x - b.x) * (a.x - b.x)
        + (a.y - b.y) * (a.y - b.y)
        + (a.z - b.z) * (a.z - b.z)
    )


def same_point(a, b, tolerance=1e-3):
    if a is None or b is None:
        return False
    return distance(a, b) <= tolerance


def same_path(a, b, tolerance=1e-3):
    if a is None or b is None or len(a) != len(b):
        return False
    return all(same_point(pa, pb, tolerance) for pa, pb in zip(a, b))


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def path_to_points(path_msg):
    return tuple(
        Point3(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        for pose in path_msg.poses
    )


class CompetitionMissionSequencer:
    def __init__(
        self,
        task_a_goal,
        transfer_goal,
        reach_radius=0.35,
        single_goal_only=False,
        update_task_a_goal_from_guide=True,
        target_ready=True,
    ):
        self.task_a_goal = Point3(*task_a_goal)
        self.transfer_goal = Point3(*transfer_goal)
        self.task_d_goal = None
        self.reach_radius = float(reach_radius)
        self.single_goal_only = bool(single_goal_only)
        self.update_task_a_goal_from_guide = bool(update_task_a_goal_from_guide)
        self.target_ready = bool(target_ready)
        self.task_a_guide_path = None
        self.task_d_guide_path = None
        self.phase = "wait_task_a_guide"

    def set_task_a_goal(self, point):
        self.task_a_goal = Point3(point.x, point.y, point.z)
        self.target_ready = True
        if self.phase == "wait_task_a_guide" and self.task_a_guide_path is not None:
            self.phase = "task_a_goal"

    def set_task_a_guide_path(self, points):
        if len(points) < 2:
            return
        self.task_a_guide_path = tuple(points)
        if self.update_task_a_goal_from_guide:
            self.task_a_goal = self.task_a_guide_path[-1]
            self.target_ready = True
        if self.phase == "wait_task_a_guide" and self.target_ready:
            self.phase = "task_a_goal"

    def set_task_d_guide_path(self, points):
        if len(points) < 2:
            return
        self.task_d_guide_path = tuple(points)
        self.task_d_goal = self.task_d_guide_path[-1]
        if self.phase == "wait_task_d_guide":
            self.phase = "task_d_goal"

    def planner_goal(self, odom_position):
        if self.phase == "wait_task_a_guide":
            return None
        if not self.target_ready:
            return None

        if self.phase == "task_a_goal":
            if distance(odom_position, self.task_a_goal) <= self.reach_radius:
                if self.single_goal_only:
                    self.phase = "task_a_hover"
                    return self.task_a_goal
                self.phase = "transfer_goal"
                return self.transfer_goal
            return self.task_a_goal

        if self.phase == "task_a_hover":
            return self.task_a_goal

        if self.phase == "transfer_goal":
            if distance(odom_position, self.transfer_goal) <= self.reach_radius:
                self.phase = "wait_task_d_guide"
            return self.transfer_goal

        if self.phase == "wait_task_d_guide":
            if self.task_d_goal is None:
                return self.transfer_goal
            self.phase = "task_d_goal"
            return self.task_d_goal

        if self.phase == "task_d_goal":
            if self.task_d_goal is not None and distance(odom_position, self.task_d_goal) <= self.reach_radius:
                self.phase = "task_d_hover"
            return self.task_d_goal

        if self.phase == "task_d_hover":
            return self.task_d_goal

        return None

    def active_guide_path(self):
        if self.phase in ("task_a_goal", "task_a_hover"):
            return self.task_a_guide_path
        if self.phase in ("transfer_goal", "wait_task_d_guide"):
            return (self.task_a_goal, self.transfer_goal)
        if self.phase in ("task_d_goal", "task_d_hover"):
            return self.task_d_guide_path
        return None


class TaskAWaypointManagerNode:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.task_a_guide_path_topic = rospy.get_param("~task_a_guide_path_topic", "/task_a/guide_path")
        self.task_a_candidate_guide_path_topic = rospy.get_param(
            "~task_a_candidate_guide_path_topic", "/task_a/candidate_guide_path"
        )
        self.use_candidate_guide_path = as_bool(rospy.get_param("~use_candidate_guide_path", True))
        self.task_d_guide_path_topic = rospy.get_param("~task_d_guide_path_topic", "/task_d/guide_path")
        self.planner_guide_path_topic = rospy.get_param("~planner_guide_path_topic", "/competition/guide_path")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.reach_radius = float(rospy.get_param("~reach_radius", 0.35))
        self.single_goal_only = as_bool(rospy.get_param("~single_goal_only", False))
        self.update_task_a_goal_from_guide = as_bool(
            rospy.get_param("~update_task_a_goal_from_guide", True)
        )
        self.target_ready = as_bool(rospy.get_param("~target_ready", True))
        self.goal_input_topic = rospy.get_param("~goal_input_topic", "/task_a/final_goal")
        self.goal_input_use_pose_z = as_bool(rospy.get_param("~goal_input_use_pose_z", False))
        self.publish_period = float(rospy.get_param("~publish_period", 0.5))
        self.goal_republish_period = float(rospy.get_param("~goal_republish_period", 0.0))
        self.require_offboard_before_goal_publish = as_bool(
            rospy.get_param("~require_offboard_before_goal_publish", False)
        )
        self.mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        task_a_goal = (
            float(rospy.get_param("~goal_x", 3.120975971221924)),
            float(rospy.get_param("~goal_y", -5.956602573394775)),
            float(rospy.get_param("~goal_z", 1.0)),
        )
        self.default_goal_z = task_a_goal[2]
        self.takeoff_before_task = as_bool(rospy.get_param("~takeoff_before_task", True))
        self.takeoff_height = float(rospy.get_param("~takeoff_height", self.default_goal_z))
        self.takeoff_reach_radius = float(rospy.get_param("~takeoff_reach_radius", 0.15))
        transfer_goal = (
            float(rospy.get_param("~transfer_x", 10.361700057983398)),
            float(rospy.get_param("~transfer_y", -5.367546081542969)),
            float(rospy.get_param("~transfer_z", 1.0)),
        )

        self.sequencer = CompetitionMissionSequencer(
            task_a_goal=task_a_goal,
            transfer_goal=transfer_goal,
            reach_radius=self.reach_radius,
            single_goal_only=self.single_goal_only,
            update_task_a_goal_from_guide=self.update_task_a_goal_from_guide,
            target_ready=self.target_ready,
        )
        self.odom_position = None
        self.last_planner_goal = None
        self.last_goal_publish_time = None
        self.last_planner_guide_path = None
        self.last_task_a_guide_path_count = 0
        self.last_task_a_candidate_guide_path_count = 0
        self.last_task_d_guide_path_count = 0
        self.takeoff_started = False
        self.takeoff_complete = not self.takeoff_before_task
        self.takeoff_goal = None
        self.takeoff_guide_path = None
        self.mavros_mode = ""
        self.mavros_state_received = False

        self.goal_publisher = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.planner_guide_path_publisher = rospy.Publisher(self.planner_guide_path_topic, Path, queue_size=1, latch=True)
        self.odom_subscriber = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.mavros_state_subscriber = None
        if self.require_offboard_before_goal_publish:
            if State is None:
                rospy.logwarn(
                    "task_a_waypoint_manager: mavros_msgs unavailable; OFFBOARD goal gate disabled"
                )
                self.require_offboard_before_goal_publish = False
            else:
                self.mavros_state_subscriber = rospy.Subscriber(
                    self.mavros_state_topic, State, self.mavros_state_callback, queue_size=1
                )
        self.goal_input_subscriber = rospy.Subscriber(
            self.goal_input_topic, PoseStamped, self.goal_input_callback, queue_size=1
        )
        self.task_a_guide_path_subscriber = rospy.Subscriber(
            self.task_a_guide_path_topic, Path, self.task_a_guide_path_callback, queue_size=1
        )
        self.task_a_candidate_guide_path_subscriber = None
        if self.use_candidate_guide_path:
            self.task_a_candidate_guide_path_subscriber = rospy.Subscriber(
                self.task_a_candidate_guide_path_topic,
                Path,
                self.task_a_candidate_guide_path_callback,
                queue_size=1,
            )
        self.task_d_guide_path_subscriber = rospy.Subscriber(
            self.task_d_guide_path_topic, Path, self.task_d_guide_path_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(self.publish_period), self.timer_callback)

        rospy.loginfo(
            "task_a_waypoint_manager: task_a_guide=%s candidate_guide=%s use_candidate=%s task_d_guide=%s planner_guide=%s goal=%s goal_input=%s target_ready=%s single_goal_only=%s update_goal_from_guide=%s require_offboard=%s state_topic=%s takeoff_before_task=%s takeoff_height=%.2f task_a=(%.2f, %.2f, %.2f) transfer=(%.2f, %.2f, %.2f)",
            self.task_a_guide_path_topic,
            self.task_a_candidate_guide_path_topic,
            self.use_candidate_guide_path,
            self.task_d_guide_path_topic,
            self.planner_guide_path_topic,
            self.goal_topic,
            self.goal_input_topic,
            self.sequencer.target_ready,
            self.single_goal_only,
            self.update_task_a_goal_from_guide,
            self.require_offboard_before_goal_publish,
            self.mavros_state_topic,
            self.takeoff_before_task,
            self.takeoff_height,
            task_a_goal[0],
            task_a_goal[1],
            task_a_goal[2],
            transfer_goal[0],
            transfer_goal[1],
            transfer_goal[2],
        )

    def odom_callback(self, odom):
        position = odom.pose.pose.position
        self.odom_position = Point3(position.x, position.y, position.z)

    def mavros_state_callback(self, state):
        self.mavros_mode = state.mode
        self.mavros_state_received = True

    def offboard_ready(self):
        if not self.require_offboard_before_goal_publish:
            return True
        return self.mavros_state_received and self.mavros_mode == "OFFBOARD"

    def goal_input_callback(self, goal):
        position = goal.pose.position
        z = position.z if self.goal_input_use_pose_z else self.default_goal_z
        if not self.goal_input_use_pose_z and abs(position.z) > 1e-3:
            z = position.z
        self.sequencer.set_task_a_goal(Point3(position.x, position.y, z))
        self.last_planner_goal = None
        self.last_planner_guide_path = None
        rospy.loginfo(
            "task_a_waypoint_manager: received RViz target goal=(%.2f, %.2f, %.2f); waiting/publishing through detected corridor",
            self.sequencer.task_a_goal.x,
            self.sequencer.task_a_goal.y,
            self.sequencer.task_a_goal.z,
        )

    def task_a_guide_path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        points = path_to_points(msg)
        self.last_task_a_guide_path_count = len(points)
        if same_path(points, self.sequencer.task_a_guide_path):
            return
        self.sequencer.set_task_a_guide_path(points)
        self.last_planner_goal = None
        rospy.loginfo(
            "task_a_waypoint_manager: received Task A guide corridor path with %d poses; task_a_goal=(%.2f, %.2f, %.2f)",
            len(points),
            self.sequencer.task_a_goal.x,
            self.sequencer.task_a_goal.y,
            self.sequencer.task_a_goal.z,
        )

    def task_a_candidate_guide_path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        points = path_to_points(msg)
        self.last_task_a_candidate_guide_path_count = len(points)
        if self.last_task_a_guide_path_count > 0:
            return
        if same_path(points, self.sequencer.task_a_guide_path):
            return
        self.sequencer.set_task_a_guide_path(points)
        self.last_planner_goal = None
        self.last_planner_guide_path = None
        rospy.loginfo_throttle(
            1.0,
            "task_a_waypoint_manager: using candidate Task A guide path for planner preview with %d poses; final guide_path will override it",
            len(points),
        )

    def task_d_guide_path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        points = path_to_points(msg)
        self.last_task_d_guide_path_count = len(points)
        if same_path(points, self.sequencer.task_d_guide_path):
            return
        self.sequencer.set_task_d_guide_path(points)
        self.last_planner_goal = None
        rospy.loginfo_once(
            "task_a_waypoint_manager: received Task D guide corridor path with %d poses",
            len(points),
        )

    def make_path_msg(self, points, stamp):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def publish_active_guide_path(self, stamp):
        active_path = self.sequencer.active_guide_path()
        if active_path is None or len(active_path) < 2:
            return
        if same_path(active_path, self.last_planner_guide_path):
            return
        self.planner_guide_path_publisher.publish(self.make_path_msg(active_path, stamp))
        self.last_planner_guide_path = active_path
        rospy.loginfo(
            "task_a_waypoint_manager: published planner guide path phase=%s poses=%d",
            self.sequencer.phase,
            len(active_path),
        )

    def task_ready_for_takeoff(self):
        return self.sequencer.target_ready and self.sequencer.phase != "wait_task_a_guide"

    def handle_takeoff_if_needed(self, stamp):
        if self.takeoff_complete:
            return False

        if not self.task_ready_for_takeoff():
            return False

        if self.odom_position.z >= self.takeoff_height - self.takeoff_reach_radius:
            self.takeoff_complete = True
            self.last_planner_goal = None
            self.last_planner_guide_path = None
            if self.takeoff_started:
                rospy.loginfo(
                    "task_a_waypoint_manager: takeoff path complete at z=%.2f; enabling Task A planner goal",
                    self.odom_position.z,
                )
            return False

        if self.takeoff_goal is None:
            self.takeoff_goal = Point3(
                self.odom_position.x,
                self.odom_position.y,
                self.takeoff_height,
            )
            self.takeoff_guide_path = (
                Point3(self.odom_position.x, self.odom_position.y, self.odom_position.z),
                self.takeoff_goal,
            )
            self.takeoff_started = True
            self.last_planner_goal = None
            self.last_planner_guide_path = None
            rospy.loginfo(
                "task_a_waypoint_manager: OFFBOARD ready; planning takeoff path from z=%.2f to z=%.2f before Task A",
                self.odom_position.z,
                self.takeoff_height,
            )

        if self.takeoff_guide_path is not None and not same_path(
            self.takeoff_guide_path, self.last_planner_guide_path
        ):
            self.planner_guide_path_publisher.publish(self.make_path_msg(self.takeoff_guide_path, stamp))
            self.last_planner_guide_path = self.takeoff_guide_path

        if not same_point(self.takeoff_goal, self.last_planner_goal):
            goal_msg = PoseStamped()
            goal_msg.header.stamp = stamp
            goal_msg.header.frame_id = self.frame_id
            goal_msg.pose.position.x = self.takeoff_goal.x
            goal_msg.pose.position.y = self.takeoff_goal.y
            goal_msg.pose.position.z = self.takeoff_goal.z
            goal_msg.pose.orientation.w = 1.0
            self.goal_publisher.publish(goal_msg)
            self.last_planner_goal = self.takeoff_goal
            self.last_goal_publish_time = stamp
            rospy.loginfo(
                "task_a_waypoint_manager: sent takeoff planner goal=(%.2f, %.2f, %.2f)",
                self.takeoff_goal.x,
                self.takeoff_goal.y,
                self.takeoff_goal.z,
            )

        rospy.loginfo_throttle(
            1.0,
            "task_a_waypoint_manager: takeoff phase active current_z=%.2f target_z=%.2f",
            self.odom_position.z,
            self.takeoff_height,
        )
        return True

    def timer_callback(self, _event):
        if self.odom_position is None:
            return

        if not self.offboard_ready():
            rospy.loginfo_throttle(
                1.0,
                "task_a_waypoint_manager: target/path ready, waiting for OFFBOARD on %s before publishing planner goal",
                self.mavros_state_topic,
            )
            return

        stamp = rospy.Time.now()
        if self.handle_takeoff_if_needed(stamp):
            return

        target = self.sequencer.planner_goal(self.odom_position)
        if target is None:
            return

        self.publish_active_guide_path(stamp)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = stamp
        goal_msg.header.frame_id = self.frame_id
        goal_msg.pose.position.x = target.x
        goal_msg.pose.position.y = target.y
        goal_msg.pose.position.z = target.z
        goal_msg.pose.orientation.w = 1.0

        should_publish_goal = False
        publish_reason = "sent"
        if not same_point(target, self.last_planner_goal):
            should_publish_goal = True
        elif (
            self.goal_republish_period > 0.0
            and self.sequencer.phase not in ("task_a_hover", "task_d_hover")
        ):
            if (
                self.last_goal_publish_time is None
                or (stamp - self.last_goal_publish_time).to_sec()
                >= self.goal_republish_period
            ):
                should_publish_goal = True
                publish_reason = "republished"

        if should_publish_goal:
            self.goal_publisher.publish(goal_msg)
            self.last_planner_goal = target
            self.last_goal_publish_time = stamp
            rospy.loginfo(
                "task_a_waypoint_manager: %s planner goal phase=%s goal=(%.2f, %.2f, %.2f)",
                publish_reason,
                self.sequencer.phase,
                target.x,
                target.y,
                target.z,
            )

        rospy.loginfo_throttle(
            1.0,
            "task_a_waypoint_manager: phase=%s task_a_guide=%d task_d_guide=%d target=(%.2f, %.2f, %.2f)",
            self.sequencer.phase,
            self.last_task_a_guide_path_count,
            self.last_task_d_guide_path_count,
            target.x,
            target.y,
            target.z,
        )


def main():
    rospy.init_node("task_a_waypoint_manager")
    TaskAWaypointManagerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
