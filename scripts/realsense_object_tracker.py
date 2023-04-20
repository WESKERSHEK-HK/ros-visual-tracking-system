#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from geometry_msgs.msg import Point

def callback(data):
    gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)

    points = np.array(list(gen))
    
    if points.size > 0:
        largest_object_index = np.argmax(points[:, 2])
        largest_object_point = points[largest_object_index]

        x, y, z = largest_object_point

        if -30 <= x <= 30 and -50 <= z <= 50:
            position = Point(x, y, z)
            pub_position.publish(position)

            if abs(x) >= 29 or abs(z) >= 49:
                pub_home.publish(Empty())

def main():
    global pub_position, pub_home

    rospy.init_node('realsense_object_tracker', anonymous=True)

    rospy.Subscriber('/camera/depth/points', PointCloud2, callback)
    pub_position = rospy.Publisher('/dog/position', Point, queue_size=1)
    pub_home = rospy.Publisher('/dog/home', Empty, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
