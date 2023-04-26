#!/usr/bin/env python3
"""
HSV-based Object Tracker Node

This node provides real-time object tracking using HSV color space filtering.
It integrates with RealSense cameras for depth-aware positioning and publishes
object position data to ROS topics.

Author: Object Tracker Developer
License: MIT
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError

from object_tracker.utils.tracking_utils import TrackingUtils
from object_tracker.utils.visualization import VisualizationUtils


class HSVObjectTracker:
    """
    HSV-based object tracker for real-time color-based object detection.
    
    This class implements a robust object tracking system using HSV color
    space filtering with configurable parameters and depth integration.
    """
    
    def __init__(self):
        """Initialize the HSV object tracker."""
        # Initialize ROS components
        self.bridge = CvBridge()
        self.depth_image = None
        
        # Initialize tracking utilities
        self.tracking_utils = TrackingUtils()
        self.viz_utils = VisualizationUtils()
        
        # ROS subscribers
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self._image_callback,
            queue_size=1
        )
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_rect_raw", 
            Image, 
            self._depth_callback,
            queue_size=1
        )
        
        # ROS publishers
        self.position_pub = rospy.Publisher(
            "/dog/position", 
            Point, 
            queue_size=10
        )
        self.home_pub = rospy.Publisher(
            "/dog/home", 
            Empty, 
            queue_size=10
        )
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize OpenCV windows and trackbars
        self._setup_ui()
        
        rospy.loginfo("HSV Object Tracker initialized successfully")
    
    def _init_parameters(self):
        """Initialize ROS parameters with defaults."""
        # HSV filtering parameters
        self.h_lower = rospy.get_param("~h_lower", 0)
        self.h_upper = rospy.get_param("~h_upper", 179)
        self.s_lower = rospy.get_param("~s_lower", 0)
        self.s_upper = rospy.get_param("~s_upper", 255)
        self.v_lower = rospy.get_param("~v_lower", 0)
        self.v_upper = rospy.get_param("~v_upper", 255)
        
        # Contour filtering parameters
        self.min_contour_size = rospy.get_param("~min_contour_size", 100)
        self.max_contour_size = rospy.get_param("~max_contour_size", 10000)
        
        # Home zone parameters
        self.home_zone_x_min = rospy.get_param("~home_zone_x_min", -30)
        self.home_zone_x_max = rospy.get_param("~home_zone_x_max", 30)
        self.home_zone_z_min = rospy.get_param("~home_zone_z_min", -50)
        self.home_zone_z_max = rospy.get_param("~home_zone_z_max", 50)
    
    def _setup_ui(self):
        """Setup OpenCV windows and trackbars for parameter adjustment."""
        cv2.namedWindow("HSV Tracker Settings", cv2.WINDOW_AUTOSIZE)
        
        # Create trackbars for HSV parameters
        cv2.createTrackbar("H Lower", "HSV Tracker Settings", self.h_lower, 179, self._h_lower_callback)
        cv2.createTrackbar("H Upper", "HSV Tracker Settings", self.h_upper, 179, self._h_upper_callback)
        cv2.createTrackbar("S Lower", "HSV Tracker Settings", self.s_lower, 255, self._s_lower_callback)
        cv2.createTrackbar("S Upper", "HSV Tracker Settings", self.s_upper, 255, self._s_upper_callback)
        cv2.createTrackbar("V Lower", "HSV Tracker Settings", self.v_lower, 255, self._v_lower_callback)
        cv2.createTrackbar("V Upper", "HSV Tracker Settings", self.v_upper, 255, self._v_upper_callback)
        
        # Create trackbars for contour filtering
        cv2.createTrackbar("Min Contour Size", "HSV Tracker Settings", self.min_contour_size, 10000, self._min_contour_callback)
        cv2.createTrackbar("Max Contour Size", "HSV Tracker Settings", self.max_contour_size, 100000, self._max_contour_callback)
    
    def _h_lower_callback(self, value):
        """Callback for H lower trackbar."""
        self.h_lower = value
    
    def _h_upper_callback(self, value):
        """Callback for H upper trackbar."""
        self.h_upper = value
    
    def _s_lower_callback(self, value):
        """Callback for S lower trackbar."""
        self.s_lower = value
    
    def _s_upper_callback(self, value):
        """Callback for S upper trackbar."""
        self.s_upper = value
    
    def _v_lower_callback(self, value):
        """Callback for V lower trackbar."""
        self.v_lower = value
    
    def _v_upper_callback(self, value):
        """Callback for V upper trackbar."""
        self.v_upper = value
    
    def _min_contour_callback(self, value):
        """Callback for min contour size trackbar."""
        self.min_contour_size = value
    
    def _max_contour_callback(self, value):
        """Callback for max contour size trackbar."""
        self.max_contour_size = value
    
    def _depth_callback(self, data):
        """
        Callback for depth image data.
        
        Args:
            data: ROS Image message containing depth data
        """
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {e}")
    
    def _image_callback(self, data):
        """
        Callback for color image data.
        
        Args:
            data: ROS Image message containing color image data
        """
        if self.depth_image is None:
            rospy.logwarn_throttle(5, "Depth image not available")
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting color image: {e}")
            return
        
        # Track object using HSV filtering
        tracking_result = self._track_object_hsv(hsv_image, cv_image)
        
        if tracking_result.is_valid():
            self._process_tracking_result(tracking_result, cv_image)
            self._publish_position(tracking_result)
            self._check_home_zone(tracking_result)
    
    def _track_object_hsv(self, hsv_image, cv_image):
        """
        Track objects using HSV color filtering.
        
        Args:
            hsv_image: HSV converted image
            cv_image: Original BGR image
            
        Returns:
            TrackingResult object containing tracking information
        """
        # Create HSV mask
        lower_bound = np.array([self.h_lower, self.s_lower, self.v_lower])
        upper_bound = np.array([self.h_upper, self.s_upper, self.v_upper])
        binary_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Find contours
        contours, _ = cv2.findContours(
            binary_mask, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filter contours by size
        filtered_contours = [
            c for c in contours 
            if self.min_contour_size <= cv2.contourArea(c) <= self.max_contour_size
        ]
        
        if not filtered_contours:
            return TrackingResult()
        
        # Find largest contour
        largest_contour = max(filtered_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        object_center = (x + w // 2, y + h // 2)
        
        return TrackingResult(
            object_center=object_center,
            bounding_box=(x, y, w, h),
            contour_area=cv2.contourArea(largest_contour),
            is_valid=True
        )
    
    def _process_tracking_result(self, tracking_result, cv_image):
        """
        Process and visualize tracking results.
        
        Args:
            tracking_result: TrackingResult object
            cv_image: Image to draw on
        """
        if not tracking_result.is_valid():
            return
        
        x, y, w, h = tracking_result.bounding_box
        
        # Draw bounding box
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Resize for display
        display_size = (320, 240)
        resized_image = cv2.resize(cv_image, display_size)
        
        # Add text overlay
        self.viz_utils.add_tracking_info(
            resized_image, 
            tracking_result, 
            display_size, 
            cv_image.shape
        )
        
        # Show result
        cv2.imshow("HSV Object Tracking", resized_image)
        cv2.waitKey(1)
    
    def _publish_position(self, tracking_result):
        """
        Publish object position to ROS topic.
        
        Args:
            tracking_result: TrackingResult object
        """
        if not tracking_result.is_valid():
            return
        
        x, y = tracking_result.object_center
        depth_image_height, depth_image_width = self.depth_image.shape
        
        # Check bounds
        if not (0 <= x < depth_image_width and 0 <= y < depth_image_height):
            rospy.logwarn_throttle(1, f"Object position ({x}, {y}) out of depth image bounds")
            return
        
        # Get depth value
        depth = self.depth_image[y, x]
        
        # Create and publish position message
        position_msg = Point()
        position_msg.x = float(x)
        position_msg.y = float(depth)
        position_msg.z = 0.0
        
        self.position_pub.publish(position_msg)
        rospy.logdebug(f"Published position: x={x}, y={depth}")
    
    def _check_home_zone(self, tracking_result):
        """
        Check if object is in home zone and publish home signal.
        
        Args:
            tracking_result: TrackingResult object
        """
        if not tracking_result.is_valid():
            return
        
        x, y = tracking_result.object_center
        depth_image_height, depth_image_width = self.depth_image.shape
        
        if not (0 <= x < depth_image_width and 0 <= y < depth_image_height):
            return
        
        depth = self.depth_image[y, x]
        
        # Check if in home zone
        in_home_zone = (
            self.home_zone_x_min <= x <= self.home_zone_x_max and
            self.home_zone_z_min <= depth <= self.home_zone_z_max
        )
        
        if in_home_zone:
            self.home_pub.publish(Empty())
            rospy.loginfo("Object detected in home zone")
    
    def run(self):
        """Main run loop."""
        rospy.loginfo("HSV Object Tracker started")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down HSV Object Tracker")
        finally:
            cv2.destroyAllWindows()


class TrackingResult:
    """Container class for tracking results."""
    
    def __init__(self, object_center=None, bounding_box=None, contour_area=None, is_valid=False):
        self.object_center = object_center
        self.bounding_box = bounding_box
        self.contour_area = contour_area
        self.is_valid = is_valid
    
    def is_valid(self):
        """Check if tracking result is valid."""
        return self.is_valid


def main():
    """Main function."""
    rospy.init_node("hsv_object_tracker", anonymous=False)
    
    try:
        tracker = HSVObjectTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt received")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
