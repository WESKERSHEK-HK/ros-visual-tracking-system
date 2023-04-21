#!/usr/bin/env python
import os
os.environ['OPENCV_UI_BACKEND'] = 'GTK2'
import rospy
import cv2
import numpy as np
import json
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from cv_bridge import CvBridge

SETTINGS_FILE = "settings.json"

class RobotTracker:
    def __init__(self):
        rospy.init_node('object_tracker', anonymous=True)
        self.bridge = CvBridge()
        self.position_pub = rospy.Publisher('/dog/position', Point, queue_size=10)
        self.home_pub = rospy.Publisher('/dog/home', Empty, queue_size=10)

        self.depth_img = None
        self.rgb_img = None

        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)

        self.rate = rospy.Rate(30)

        cv2.waitKey(1)
        print('start robot tracker')
        # Load settings from file
        self.settings = self.load_settings()

        # Create UI window and trackbars
        cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Color Threshold", "Settings", self.settings["color_threshold"], 255, self.update_settings)
        cv2.createTrackbar("Contour Size", "Settings", self.settings["contour_size"], 1000, self.update_settings)
        cv2.createTrackbar("Trim Size", "Settings", self.settings["trim_size"], 50, self.update_settings)
        print('start create cv window')

    def update_settings(self, x):
        self.settings["color_threshold"] = cv2.getTrackbarPos("Color Threshold", "Settings")
        self.settings["contour_size"] = cv2.getTrackbarPos("Contour Size", "Settings")
        self.settings["trim_size"] = cv2.getTrackbarPos("Trim Size", "Settings")
        self.save_settings()

    def load_settings(self):
        SETTINGS_FILE = "settings.json"  # Define the settings file name

        try:
        # Try to open and read the settings file
            with open(SETTINGS_FILE, "r") as f:
                settings = json.load(f)
        except IOError:
        # If an error occurs (e.g., file not found), use default settings
            settings = {
                "color_threshold": 30,
                "contour_size": 100,
                "trim_size": 10
            }
        return settings  # Return the loaded or default settings


    def save_settings(self):
        with open(SETTINGS_FILE, "w") as f:
            json.dump(self.settings, f)

    def rgb_callback(self, msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def track_robot(self):
        if self.rgb_img is None or self.depth_img is None:
            return None
        print("RGB image shape:", self.rgb_img.shape)
        print("Depth image shape:", self.depth_img.shape)

        self.rgb_img = cv2.cvtColor(self.rgb_img, cv2.COLOR_RGB2BGR)  # Convert color image to BGR format

        cv2.imshow("Color Image", self.rgb_img)
        cv2.imshow("Depth Image", self.depth_img)

        # Get trackbar values from settings
        color_threshold = self.settings["color_threshold"]
        contour_size = self.settings["contour_size"]
        trim_size = self.settings["trim_size"]

        # Crop the images based on trim size
        height, width = self.rgb_img.shape[:2]
        left_trim = int((trim_size / 100) * width)
        right_trim = int((1 - trim_size / 100) * width)
        self.rgb_img = self.rgb_img[:, left_trim:right_trim]
        self.depth_img = self.depth_img[:, left_trim:right_trim]

        # Threshold black color with the specified threshold
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([color_threshold, color_threshold, color_threshold])
        mask = cv2.inRange(self.rgb_img, lower_black, upper_black)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour with the specified contour size
        max_area = contour_size
        robot_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                robot_contour = contour

        if robot_contour is not None:
            # Calculate the centroid of the contour
            moments = cv2.moments(robot_contour)
            cx = int(moments['m10'] / moments['m00'])
            cz = int(moments['m01'] / moments['m00'])

            # Get depth from the depth image
            depth = self.depth_img[cz, cx]

            # Publish the position
            position = Point()
            position.x = cx + left_trim  # Add the left_trim to the x-coordinate
            position.y = depth
            self.position_pub.publish(position)

            # Check if the robot is near the limit            limit = int((1 - 2 * trim_size / 100) * width)
            if cx > limit:
                self.home_pub.publish(Empty())

    def run(self):
        while not rospy.is_shutdown():
            self.track_robot()
            self.rate.sleep()
            cv2.waitKey(1)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        robot_tracker = RobotTracker()
        robot_tracker.run()
    except rospy.ROSInterruptException:
        pass
