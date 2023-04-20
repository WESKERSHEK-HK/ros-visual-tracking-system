#!/usr/bin/env python

import rospy
from realsense2_camera.msg import CustomVector
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Empty

class ObjectTracker:
    def __init__(self):
        rospy.init_node('object_tracker')

        self.largest_object_pub = rospy.Publisher('/dog/position', Point, queue_size=10)
        self.home_command_pub = rospy.Publisher('/dog/home', Empty, queue_size=10)


        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)

        self.max_x = 30
        self.min_x = -30
        self.max_z = 50
        self.min_z = -50

        rospy.spin()

    def pointcloud_callback(self, point_cloud):
        points = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)

        largest_object = None
        largest_object_volume = -1

        for point in points:
            x, y, z = point
            volume = x * y * z

            if largest_object is None or volume > largest_object_volume:
                if self.min_x < x < self.max_x and self.min_z < z < self.max_z:
                    largest_object = point
                    largest_object_volume = volume

        if largest_object is not None:
            x, y, z = largest_object
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = z

            self.largest_object_pub.publish(point_msg)

            if x > self.max_x - 0.5 or x < self.min_x + 0.5 or z > self.max_z - 0.5 or z < self.min_z + 0.5:
                empty_msg = Empty()
                self.home_command_pub.publish(empty_msg)

if __name__ == '__main__':
    try:
        ObjectTracker()
    except rospy.ROSInterruptException:
        pass
