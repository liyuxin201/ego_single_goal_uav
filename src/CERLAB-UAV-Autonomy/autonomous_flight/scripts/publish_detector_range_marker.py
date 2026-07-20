#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class DetectorRangeMarkerPublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.marker_topic = rospy.get_param("~marker_topic", "/onboard_detector/detector_range_marker")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.range_x = float(rospy.get_param("~range_x", 20.0))
        self.range_y = float(rospy.get_param("~range_y", 20.0))
        self.range_z = float(rospy.get_param("~range_z", 5.0))
        self.alpha = float(rospy.get_param("~alpha", 0.12))

        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

        rospy.loginfo(
            "[autonomous_flight] Detector range marker: topic=%s, range=(%.2f, %.2f, %.2f)",
            self.marker_topic,
            self.range_x,
            self.range_y,
            self.range_z,
        )

    def odom_cb(self, odom):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "detector_range"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = odom.pose.pose.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * self.range_x
        marker.scale.y = 2.0 * self.range_y
        marker.scale.z = 2.0 * self.range_z
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = self.alpha
        marker.lifetime = rospy.Duration(0.3)

        self.marker_pub.publish(marker)


def main():
    rospy.init_node("publish_detector_range_marker")
    DetectorRangeMarkerPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
