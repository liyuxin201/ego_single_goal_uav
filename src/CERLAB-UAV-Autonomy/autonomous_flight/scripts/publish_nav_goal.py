#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class NavGoalPublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/CERLAB/quadcopter/odom")
        self.goal_x = float(rospy.get_param("~goal_x", 8.0))
        self.goal_y = float(rospy.get_param("~goal_y", 0.0))
        self.goal_z = float(rospy.get_param("~goal_z", 0.0))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", 0.0))
        self.ready_z = float(rospy.get_param("~ready_z", 0.8))
        self.delay = float(rospy.get_param("~delay", 1.0))

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        self.odom = None
        self.ready_since = None
        self.sent = False

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def odom_cb(self, odom):
        self.odom = odom

    def timer_cb(self, _event):
        if self.sent or self.odom is None:
            return

        curr_z = self.odom.pose.pose.position.z
        if curr_z < self.ready_z:
            self.ready_since = None
            return

        now = rospy.Time.now()
        if self.ready_since is None:
            self.ready_since = now
            return

        if (now - self.ready_since).to_sec() < self.delay:
            return

        goal = PoseStamped()
        goal.header.stamp = now
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = self.goal_z
        quat = quaternion_from_euler(0.0, 0.0, self.goal_yaw)
        goal.pose.orientation = Quaternion(*quat)
        self.goal_pub.publish(goal)
        self.sent = True
        rospy.loginfo(
            "[autonomous_flight] Published automatic navigation goal: (%.2f, %.2f, %.2f), yaw=%.2f",
            self.goal_x,
            self.goal_y,
            self.goal_z,
            self.goal_yaw,
        )


def main():
    rospy.init_node("publish_nav_goal")
    NavGoalPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
