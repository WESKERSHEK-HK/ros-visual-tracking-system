"""
Tracking Utilities Module

This module provides utility functions for object tracking operations.
"""

import cv2
import numpy as np
from typing import Tuple, List, Optional


class TrackingUtils:
    """Utility class for tracking operations."""
    
    @staticmethod
    def filter_contours_by_size(
        contours: List[np.ndarray], 
        min_area: float, 
        max_area: float
    ) -> List[np.ndarray]:
        """
        Filter contours by area size.
        
        Args:
            contours: List of contours to filter
            min_area: Minimum contour area
            max_area: Maximum contour area
            
        Returns:
            Filtered list of contours
        """
        return [
            c for c in contours 
            if min_area <= cv2.contourArea(c) <= max_area
        ]
    
    @staticmethod
    def find_largest_contour(contours: List[np.ndarray]) -> Optional[np.ndarray]:
        """
        Find the largest contour by area.
        
        Args:
            contours: List of contours
            
        Returns:
            Largest contour or None if list is empty
        """
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)
    
    @staticmethod
    def get_contour_center(contour: np.ndarray) -> Tuple[int, int]:
        """
        Calculate the center point of a contour.
        
        Args:
            contour: Contour to process
            
        Returns:
            (x, y) center coordinates
        """
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0
        return (cx, cy)
    
    @staticmethod
    def create_hsv_mask(
        image: np.ndarray, 
        lower_bound: np.ndarray, 
        upper_bound: np.ndarray
    ) -> np.ndarray:
        """
        Create a binary mask using HSV color range.
        
        Args:
            image: HSV image
            lower_bound: Lower HSV bounds [H, S, V]
            upper_bound: Upper HSV bounds [H, S, V]
            
        Returns:
            Binary mask
        """
        return cv2.inRange(image, lower_bound, upper_bound)
    
    @staticmethod
    def apply_morphological_operations(
        mask: np.ndarray, 
        kernel_size: int = 5
    ) -> np.ndarray:
        """
        Apply morphological operations to clean up mask.
        
        Args:
            mask: Binary mask to process
            kernel_size: Size of morphological kernel
            
        Returns:
            Processed mask
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # Remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Fill holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
