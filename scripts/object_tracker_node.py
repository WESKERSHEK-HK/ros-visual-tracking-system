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
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.position_pub = rospy.Publisher("/dog/position", Point, queue_size=10)
        self.home_pub = rospy.Publisher("/dog/home", Empty, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        tracked_object, object_pos = self.track_largest_black_object(cv_image)
        if tracked_object is not None:
            cv2.imshow("Tracked Object", tracked_object)
            cv2.waitKey(1)

            position_msg = Point()
            position_msg.x = object_pos[0]
            position_msg.y = 0
            position_msg.z = object_pos[1]

            self.position_pub.publish(position_msg)

            if -30 <= position_msg.x <= 30 and -50 <= position_msg.z <= 50:
                self.home_pub.publish(Empty())

    def track_largest_black_object(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None

        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(image, "X: {}, Z: {}".format(x+w//2, y+h//2), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image, (x + w // 2, y + h // 2)

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
