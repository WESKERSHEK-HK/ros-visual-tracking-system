#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageSubtraction:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.background_image = cv2.imread("background.jpeg")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        difference = cv2.absdiff(self.background_image, cv_image)
        gray = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            closest_contour = min(contours, key=lambda cnt: cv2.pointPolygonTest(cnt, (data.width/2, data.height/2), True))

            cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)
            cv2.drawContours(cv_image, [closest_contour], -1, (0, 0, 255), 3)

            # Display the resulting image
            cv2.imshow("Image Subtraction", cv_image)
            cv2.waitKey(1)

def main():
    rospy.init_node("realsense_image_subtraction", anonymous=True)
    image_subtraction = ImageSubtraction()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows