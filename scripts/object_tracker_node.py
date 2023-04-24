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
        global z
        self.bridge = CvBridge()
        self.depth_image = None
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.position_pub = rospy.Publisher("/dog/position", Point, queue_size=1000)
        self.home_pub = rospy.Publisher("/dog/home", Empty, queue_size=10)
        z = 0

        #cv2.namedWindow("Settings")
        #cv2.createTrackbar("Min Contour Size", "Settings", 100, 10000, lambda x: None)
        #cv2.createTrackbar("Max Contour Size", "Settings", 1000, 100000, lambda x: None)

    def crop_center(self, image, target_width, target_height):
        height, width = image.shape[:2]
        x_start = (width - target_width) //2
        y_start = (height - target_height) //2
        return image[x_start:y_start + target_height, x_start:y_start + target_width]

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            #self.depth_image = self.crop_center(ori_depth_image, 400, 400)
        except CvBridgeError as e:
            print(e)
            return

    def image_callback(self, data):
        global z
        if self.depth_image is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)
            return

        tracked_object, object_pos, pos, h, w = self.track_largest_black_object(hsv_image)

        if tracked_object is not None and object_pos is not None and pos is not None:
            x, y = pos
            smaller_size = (320, 240)
            tracked_object_resized = cv2.resize(tracked_object, smaller_size)
            #tracked_object_resized = tracked_object

            scaled_x = x * smaller_size[0] // cv_image.shape[1]
            scaled_y = (y + h) * smaller_size[1] // cv_image.shape[0]  # Add the height of the bounding box (h) to the y coordinate
            #scaled_x = x
            #scaled_y = y

            cv2.putText(tracked_object_resized, "X: {}, Z: {}".format(object_pos[0], object_pos[1]), (scaled_x, scaled_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(tracked_object_resized, "SizeX: {}".format(w), (scaled_x, scaled_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(tracked_object_resized, "SizeY: {}".format(h), (scaled_x, scaled_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            

            

            x, y = object_pos
            crop_depth = self.depth_image[180:720 - 270, 320:1280 - 320]
            depth_image_height, depth_image_width = crop_depth.shape
            position_msg = Point()
            position_msg.x = x
            position_msg.y = y
            position_msg.z = 0.0

            cv2.imshow("Tracked Object", tracked_object_resized)
            cv2.waitKey(1)

            # Check if x and y are within the depth image dimensions
            if 0 <= x < depth_image_width and 0 <= y < depth_image_height:
                depth = crop_depth[y, x]

                position_msg = Point()
                position_msg.x = x
                position_msg.y = y
                position_msg.z = depth
                z = depth

                

                #if position_msg.x >= 700 or position_msg.x <= 600:
                    #self.home_pub.publish(Empty())
            if position_msg.z == 0.0:
                position_msg.z = z
            self.position_pub.publish(position_msg)


    def track_largest_black_object(self, image):

        #height, width = image.shape[:2]
        #print(height)
        #print(width)
        crop_image = image[180:720 - 270, 320:1280 - 320]
        #hsv_image = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)

        # Convert the image to grayscale
        #gray = cv2.cvtColor(crop_image, cv2.COLOR_BGR2GRAY)
        
     
        # Threshold the image to create a binary image
        #_, binary = cv2.threshold(gray, 8, 255, cv2.THRESH_BINARY_INV)
        lower_bound = np.array([0, 0, 0])
        upper_bound = np.array([50, 70, 30])
        _, binary = cv2.inRange(crop_image, lower_bound, upper_bound)

        # Find contours in the binary image
        _, contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # Get slider values
        #min_contour_size = cv2.getTrackbarPos("Min Contour Size", "Settings")
        #max_contour_size = cv2.getTrackbarPos("Max Contour Size", "Settings")
        min_contour_size = 2000
        max_contour_size = 6000

        # Filter the contours by size
        filtered_contours = [c for c in contours if min_contour_size <= cv2.contourArea(c) <= max_contour_size]

        # Find the largest contour
        if len(filtered_contours) > 0:
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(binary, (x, y), (x + w, y + h), (0, 255, 0), 2)
            object_center = (x + w // 2, y + h // 2)
            return binary, object_center, (x, y), h, w
        else:
            return binary, None, None, None, None  # Return None for object_center and (x, y) when there are no valid contours



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
