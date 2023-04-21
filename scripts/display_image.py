#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    cv2.imshow("RealSenseImage", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('realsense_image_display', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()

    # Close the display window when ROS shuts down
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
