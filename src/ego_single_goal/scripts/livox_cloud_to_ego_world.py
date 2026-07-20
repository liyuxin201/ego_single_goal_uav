#!/usr/bin/env python3

import math

import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def _quat_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _rotate(matrix, point):
    x, y, z = point
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


class LivoxCloudToEgoWorld:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/livox/lidar")
        output_topic = rospy.get_param("~output_topic", "/ego/livox_cloud_world")
        odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self._frame_id = rospy.get_param("~frame_id", "map")
        self._downsample = max(1, int(rospy.get_param("~downsample", 1)))
        self._sensor_offset = (
            float(rospy.get_param("~sensor_offset_x", 0.0)),
            float(rospy.get_param("~sensor_offset_y", 0.0)),
            float(rospy.get_param("~sensor_offset_z", 0.10)),
        )

        self._odom = None
        self._publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=2)
        self._odom_subscriber = rospy.Subscriber(
            odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=20,
        )
        self._cloud_subscriber = rospy.Subscriber(
            input_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
            buff_size=16777216,
        )

        rospy.loginfo(
            "Livox cloud transformer: %s + %s -> %s, downsample=%d",
            input_topic,
            odom_topic,
            output_topic,
            self._downsample,
        )

    def _odom_callback(self, odom):
        self._odom = odom

    def _cloud_callback(self, cloud):
        if self._odom is None:
            return

        pose = self._odom.pose.pose
        rotation = _quat_to_matrix(pose.orientation)
        position = pose.position
        offset = self._sensor_offset

        transformed = []
        fields = cloud.fields
        step = self._downsample
        for index, point in enumerate(pc2.read_points(cloud, skip_nans=True)):
            if index % step:
                continue

            lx = point[0] + offset[0]
            ly = point[1] + offset[1]
            lz = point[2] + offset[2]
            wx, wy, wz = _rotate(rotation, (lx, ly, lz))

            row = list(point)
            row[0] = wx + position.x
            row[1] = wy + position.y
            row[2] = wz + position.z
            transformed.append(row)

        header = cloud.header
        header.frame_id = self._frame_id
        output = pc2.create_cloud(header, fields, transformed)
        output.is_dense = cloud.is_dense
        self._publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("livox_cloud_to_ego_world")
    LivoxCloudToEgoWorld()
    rospy.spin()
