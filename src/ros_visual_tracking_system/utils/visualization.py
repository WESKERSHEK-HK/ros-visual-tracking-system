"""
Visualization Utilities Module

This module provides utility functions for visualizing tracking results.
"""

import cv2
import numpy as np
from typing import Tuple


class VisualizationUtils:
    """Utility class for visualization operations."""
    
    @staticmethod
    def add_tracking_info(
        image: np.ndarray, 
        tracking_result, 
        display_size: Tuple[int, int], 
        original_size: Tuple[int, int]
    ) -> None:
        """
        Add tracking information overlay to image.
        
        Args:
            image: Image to add overlay to
            tracking_result: TrackingResult object
            display_size: Size of display image (width, height)
            original_size: Size of original image (height, width)
        """
        if not tracking_result.is_valid:
            return
        
        x, y, w, h = tracking_result.bounding_box
        
        # Scale coordinates for display
        scaled_x = int(x * display_size[0] / original_size[1])
        scaled_y = int((y + h) * display_size[1] / original_size[0])
        
        # Add position text
        cv2.putText(
            image, 
            f"X: {tracking_result.object_center[0]}, Z: {tracking_result.object_center[1]}", 
            (scaled_x, scaled_y + 10), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            (0, 255, 0), 
            2
        )
        
        # Add size text
        cv2.putText(
            image, 
            f"Size: {tracking_result.contour_area}", 
            (scaled_x, scaled_y + 25), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            (0, 255, 0), 
            2
        )
    
    @staticmethod
    def draw_bounding_box(
        image: np.ndarray, 
        bounding_box: Tuple[int, int, int, int], 
        color: Tuple[int, int, int] = (0, 255, 0), 
        thickness: int = 2
    ) -> None:
        """
        Draw bounding box on image.
        
        Args:
            image: Image to draw on
            bounding_box: (x, y, width, height) bounding box
            color: BGR color tuple
            thickness: Line thickness
        """
        x, y, w, h = bounding_box
        cv2.rectangle(image, (x, y), (x + w, y + h), color, thickness)
    
    @staticmethod
    def draw_center_point(
        image: np.ndarray, 
        center: Tuple[int, int], 
        color: Tuple[int, int, int] = (0, 0, 255), 
        radius: int = 5
    ) -> None:
        """
        Draw center point on image.
        
        Args:
            image: Image to draw on
            center: (x, y) center coordinates
            color: BGR color tuple
            radius: Circle radius
        """
        cv2.circle(image, center, radius, color, -1)
    
    @staticmethod
    def create_status_overlay(
        image: np.ndarray, 
        status_text: str, 
        position: Tuple[int, int] = (10, 30)
    ) -> None:
        """
        Add status text overlay to image.
        
        Args:
            image: Image to add overlay to
            status_text: Text to display
            position: (x, y) position for text
        """
        cv2.putText(
            image, 
            status_text, 
            position, 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.7, 
            (255, 255, 255), 
            2
        )
        # Add black outline for better visibility
        cv2.putText(
            image, 
            status_text, 
            position, 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.7, 
            (0, 0, 0), 
            4
        )
    
    @staticmethod
    def resize_image(
        image: np.ndarray, 
        target_size: Tuple[int, int], 
        interpolation: int = cv2.INTER_AREA
    ) -> np.ndarray:
        """
        Resize image to target size.
        
        Args:
            image: Image to resize
            target_size: (width, height) target size
            interpolation: Interpolation method
            
        Returns:
            Resized image
        """
        return cv2.resize(image, target_size, interpolation=interpolation)
