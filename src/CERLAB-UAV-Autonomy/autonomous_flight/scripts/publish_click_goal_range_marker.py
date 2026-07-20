#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class ClickGoalRangeMarkerPublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.marker_topic = rospy.get_param("~marker_topic", "/autonomous_flight/click_goal_range_marker")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.range_x = float(rospy.get_param("~range_x", 10.0))
        self.range_y = float(rospy.get_param("~range_y", 10.0))
        self.height = float(rospy.get_param("~height", 1.0))
        self.line_width = float(rospy.get_param("~line_width", 0.06))

        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

        rospy.loginfo(
            "[autonomous_flight] Click goal range marker: topic=%s, range=(+/-%.2f, +/-%.2f), height=%.2f",
            self.marker_topic,
            self.range_x,
            self.range_y,
            self.height,
        )

    def odom_cb(self, odom):
        cx = odom.pose.pose.position.x
        cy = odom.pose.pose.position.y
        z = self.height

        corners = [
            (cx - self.range_x, cy - self.range_y, z),
            (cx + self.range_x, cy - self.range_y, z),
            (cx + self.range_x, cy + self.range_y, z),
            (cx - self.range_x, cy + self.range_y, z),
        ]

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "click_goal_range"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0.3)

        for start, end in zip(corners, corners[1:] + corners[:1]):
            p0 = Point()
            p0.x, p0.y, p0.z = start
            p1 = Point()
            p1.x, p1.y, p1.z = end
            marker.points.append(p0)
            marker.points.append(p1)

        self.marker_pub.publish(marker)


def main():
    rospy.init_node("publish_click_goal_range_marker")
    ClickGoalRangeMarkerPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
