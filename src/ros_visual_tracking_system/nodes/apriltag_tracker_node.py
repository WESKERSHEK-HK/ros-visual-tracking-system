#!/usr/bin/env python3
"""
AprilTag Object Tracker Node

This node provides real-time object tracking using AprilTag fiducial markers.
It integrates with RealSense cameras for depth-aware positioning and publishes
object position data to ROS topics.

Author: Object Tracker Developer
License: MIT
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import threading
from typing import Optional, Tuple


class AprilTagTracker:
    """
    AprilTag-based object tracker for precise marker detection.
    
    This class implements a robust object tracking system using AprilTag
    fiducial markers with depth integration and fallback positioning.
    """
    
    def __init__(self):
        """Initialize the AprilTag tracker."""
        # Initialize ROS components
        self.bridge = CvBridge()
        
        # Data storage
        self.depth_data: Optional[np.ndarray] = None
        self.color_image: Optional[np.ndarray] = None
        
        # Tracking state
        self.tag_detected: bool = False
        self.tag_detect_fail_count: int = 0
        
        # Configuration parameters
        self._init_parameters()
        
        # ROS subscribers
        self.tag_sub = rospy.Subscriber(
            "/tag_detections", 
            AprilTagDetectionArray, 
            self._tag_callback,
            queue_size=1
        )
        self.depth_sub = rospy.Subscriber(
            "/camera/depth/image_rect_raw", 
            Image, 
            self._depth_callback,
            queue_size=1
        )
        self.color_sub = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self._color_callback,
            queue_size=1
        )
        
        # ROS publishers
        self.position_pub = rospy.Publisher(
            "/dog/position", 
            Point, 
            queue_size=1
        )
        
        # Visualization settings
        self.resize_factor = rospy.get_param("~resize_factor", 0.5)
        self.bbox_color = tuple(rospy.get_param("~bbox_color", [0, 255, 0]))
        self.bbox_thickness = rospy.get_param("~bbox_thickness", 2)
        
        # Start visualization thread
        self._start_visualization_thread()
        
        rospy.loginfo("AprilTag Tracker initialized successfully")
    
    def _init_parameters(self):
        """Initialize ROS parameters with defaults."""
        # Fallback parameters
        self.fallback_x = rospy.get_param("~fallback_x", 50.0)
        self.fallback_y = rospy.get_param("~fallback_y", 50.0)
        self.fallback_z = rospy.get_param("~fallback_z", 50.0)
        
        # Detection parameters
        self.max_fail_count = rospy.get_param("~max_fail_count", 100)
        self.tag_id_to_track = rospy.get_param("~tag_id_to_track", 0)
        
        # Visualization parameters
        self.show_visualization = rospy.get_param("~show_visualization", True)
    
    def _tag_callback(self, data: AprilTagDetectionArray):
        """
        Callback for AprilTag detection data.
        
        Args:
            data: AprilTag detection array message
        """
        if not data.detections or self.depth_data is None or self.color_image is None:
            self.tag_detected = False
            self._handle_tag_detection_failure()
            return
        
        # Find the specified tag ID
        target_tag = None
        for detection in data.detections:
            if detection.id[0] == self.tag_id_to_track:
                target_tag = detection
                break
        
        if target_tag is None:
            self.tag_detected = False
            self._handle_tag_detection_failure()
            return
        
        self.tag_detected = True
        self.tag_detect_fail_count = 0
        
        # Extract tag position
        tag_x = target_tag.pose.pose.pose.position.x
        tag_y = target_tag.pose.pose.pose.position.y
        
        # Get depth at tag position
        depth = self._get_depth_at_position(tag_x, tag_y)
        
        if depth is not None:
            # Publish position
            self._publish_position(tag_x, tag_y, depth)
            
            # Log detection
            rospy.logdebug(f"Tag {self.tag_id_to_track} detected at ({tag_x:.2f}, {tag_y:.2f}, {depth:.2f})")
    
    def _depth_callback(self, data: Image):
        """
        Callback for depth image data.
        
        Args:
            data: ROS Image message containing depth data
        """
        try:
            self.depth_data = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {e}")
    
    def _color_callback(self, data: Image):
        """
        Callback for color image data.
        
        Args:
            data: ROS Image message containing color image data
        """
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Error converting color image: {e}")
    
    def _get_depth_at_position(self, x: float, y: float) -> Optional[float]:
        """
        Get depth value at specified position.
        
        Args:
            x: Normalized x coordinate (0-1)
            y: Normalized y coordinate (0-1)
            
        Returns:
            Depth value in meters or None if invalid
        """
        if self.depth_data is None:
            return None
        
        # Convert normalized coordinates to pixel coordinates
        u = int(x * self.depth_data.shape[1])
        v = int(y * self.depth_data.shape[0])
        
        # Check bounds
        if not (0 <= u < self.depth_data.shape[1] and 0 <= v < self.depth_data.shape[0]):
            rospy.logwarn(f"Position ({x}, {y}) out of depth image bounds")
            return None
        
        # Get depth value and convert to meters
        depth = self.depth_data[v, u] / 1000.0
        
        # Check for invalid depth values
        if np.isnan(depth) or depth <= 0:
            rospy.logwarn(f"Invalid depth value at ({u}, {v}): {depth}")
            return None
        
        return depth
    
    def _handle_tag_detection_failure(self):
        """Handle tag detection failures with fallback positioning."""
        self.tag_detect_fail_count += 1
        
        if self.tag_detect_fail_count >= self.max_fail_count:
            rospy.logwarn(f"Tag detection failed {self.max_fail_count} times, using fallback position")
            self._publish_fallback_position()
            self.tag_detect_fail_count = 0
    
    def _publish_position(self, x: float, y: float, z: float):
        """
        Publish object position to ROS topic.
        
        Args:
            x: X coordinate
            y: Y coordinate  
            z: Z coordinate (depth)
        """
        position_msg = Point()
        position_msg.x = float(x)
        position_msg.y = float(y)
        position_msg.z = float(z)
        
        self.position_pub.publish(position_msg)
    
    def _publish_fallback_position(self):
        """Publish fallback position when tag detection fails."""
        rospy.loginfo("Publishing fallback position")
        self._publish_position(self.fallback_x, self.fallback_y, self.fallback_z)
    
    def _start_visualization_thread(self):
        """Start visualization thread if enabled."""
        if not self.show_visualization:
            return
        
        self.visualization_thread = threading.Thread(target=self._visualization_loop)
        self.visualization_thread.daemon = True
        self.visualization_thread.start()
    
    def _visualization_loop(self):
        """Main visualization loop."""
        while not rospy.is_shutdown():
            if self.color_image is not None:
                self._show_tracking_visualization()
            rospy.sleep(0.033)  # ~30 FPS
    
    def _show_tracking_visualization(self):
        """Display tracking visualization."""
        if self.color_image is None:
            return
        
        # Create copy for visualization
        img = self.color_image.copy()
        
        # Resize for display
        display_img = cv2.resize(
            img, 
            None, 
            fx=self.resize_factor, 
            fy=self.resize_factor, 
            interpolation=cv2.INTER_AREA
        )
        
        # Add status overlay
        status_text = f"Tag {self.tag_id_to_track}: {'DETECTED' if self.tag_detected else 'NOT DETECTED'}"
        self._add_status_overlay(display_img, status_text)
        
        # Show image
        cv2.imshow("AprilTag Tracking", display_img)
        cv2.waitKey(1)
    
    def _add_status_overlay(self, image: np.ndarray, text: str):
        """
        Add status text overlay to image.
        
        Args:
            image: Image to add overlay to
            text: Text to display
        """
        # Add black background
        cv2.rectangle(image, (10, 10), (300, 40), (0, 0, 0), -1)
        
        # Add text
        cv2.putText(
            image, 
            text, 
            (15, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.6, 
            (255, 255, 255), 
            2
        )
    
    def run(self):
        """Main run loop."""
        rospy.loginfo("AprilTag Tracker started")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down AprilTag Tracker")
        finally:
            if self.show_visualization:
                cv2.destroyAllWindows()


def main():
    """Main function."""
    rospy.init_node("apriltag_tracker", anonymous=False)
    
    try:
        tracker = AprilTagTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS interrupt received")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
