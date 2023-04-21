#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

class ObjectTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.position_pub = rospy.Publisher("/dog/position", Point, queue_size=10)
        self.home_pub = rospy.Publisher("/dog/home", Empty, queue_size=10)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

    def image_callback(self, data):
        if self.depth_image is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        tracked_object, object_pos, (x, y) = self.track_largest_black_object(cv_image)
        if tracked_object is not None:
            smaller_size = (320, 240)
            tracked_object_resized = cv2.resize(tracked_object, smaller_size)

            scaled_x = x * smaller_size[0] // cv_image.shape[1]
            scaled_y = y * smaller_size[1] // cv_image.shape[0]
            cv2.putText(tracked_object_resized, "X: {}, Z: {}".format(object_pos[0], object_pos[1]), (scaled_x, scaled_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Tracked Object", tracked_object_resized)
            cv2.waitKey(1)

            x, y = object_pos
            depth_image_height, depth_image_width = self.depth_image.shape

            # Check if x and y are within the depth image dimensions
            if 0 <= x < depth_image_width and 0 <= y < depth_image_height:
                depth = self.depth_image[y, x]

                position_msg = Point()
                position_msg.x = x
                position_msg.y = 0
                position_msg.z = depth

                self.position_pub.publish(position_msg)

                if -30 <= position_msg.x <= 30 and -50 <= position_msg.z <= 50:
                    self.home_pub.publish(Empty())



    def track_largest_black_object(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY_INV)

        contours = None
        if cv2.__version__.startswith('3'):
            _, contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, None

        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        #cv2.putText(image, "X: {}, Z: {}".format(x+w//2, y+h//2), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image, (x + w // 2, y + h // 2), (x, y)


def main():
    rospy.init_node("object_tracker_node", anonymous=True)
    object_tracker = ObjectTracker()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
