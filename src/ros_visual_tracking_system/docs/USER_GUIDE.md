# Object Tracker User Guide

## Table of Contents
1. [Introduction](#introduction)
2. [System Overview](#system-overview)
3. [Getting Started](#getting-started)
4. [HSV Tracking](#hsv-tracking)
5. [AprilTag Tracking](#apriltag-tracking)
6. [Advanced Configuration](#advanced-configuration)
7. [Troubleshooting](#troubleshooting)
8. [API Reference](#api-reference)

## Introduction

The Object Tracker is a sophisticated ROS package designed for real-time object tracking in robotic applications. It provides two primary tracking methods:

- **HSV Color-based Tracking**: Detects objects based on color characteristics
- **AprilTag Fiducial Tracking**: Detects and tracks fiducial markers for precise positioning

This guide will walk you through setting up, configuring, and using the system effectively.

## System Overview

### Architecture
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   RealSense    │    │   Object         │    │   ROS Topics    │
│   Camera       │───▶│   Tracker        │───▶│   /dog/position │
│                │    │   System         │    │   /dog/home     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Key Components
- **Camera Interface**: RealSense D400 series integration
- **Tracking Engine**: Dual-mode object detection
- **Position Publisher**: 3D coordinate output
- **Home Zone Monitor**: Configurable boundary detection
- **Visualization Tools**: Real-time tracking display

## Getting Started

### Prerequisites
1. ROS Melodic or Noetic installed
2. RealSense camera connected
3. Python 3.6+ with OpenCV
4. Required ROS packages installed

### Quick Launch
```bash
# Basic HSV tracking
roslaunch object_tracker hsv_tracker.launch

# Basic AprilTag tracking
roslaunch object_tracker apriltag_tracker.launch

# Combined system
roslaunch object_tracker combined_tracker.launch
```

### Verify System Status
```bash
# Check camera topics
rostopic list | grep camera

# Monitor tracking output
rostopic echo /dog/position

# Check node status
rosnode list
rosnode info /hsv_object_tracker
```

## HSV Tracking

### Understanding HSV Color Space
HSV (Hue, Saturation, Value) is ideal for color-based tracking:

- **Hue (H)**: Color type (0-179 in OpenCV)
- **Saturation (S)**: Color purity (0-255)
- **Value (V)**: Color brightness (0-255)

### Configuration Methods

#### 1. Interactive Sliders
Launch the HSV tracker and use the on-screen sliders:
```bash
roslaunch object_tracker hsv_tracker.launch
```

Adjust parameters in real-time:
- H Lower/Upper: Define color range
- S Lower/Upper: Define saturation range
- V Lower/Upper: Define brightness range
- Min/Max Contour Size: Filter object size

#### 2. ROS Parameters
Set parameters programmatically:
```bash
# Set HSV bounds for black objects
rosparam set /hsv_object_tracker/h_lower 0
rosparam set /hsv_object_tracker/h_upper 179
rosparam set /hsv_object_tracker/s_lower 0
rosparam set /hsv_object_tracker/s_upper 255
rosparam set /hsv_object_tracker/v_lower 0
rosparam set /hsv_object_tracker/v_upper 50

# Set contour filtering
rosparam set /hsv_object_tracker/min_contour_size 100
rosparam set /hsv_object_tracker/max_contour_size 10000
```

#### 3. Configuration File
Edit `config/hsv_tracker_params.yaml`:
```yaml
hsv_tracker:
  # HSV bounds for black objects
  h_lower: 0
  h_upper: 179
  s_lower: 0
  s_upper: 255
  v_lower: 0
  v_upper: 50
  
  # Contour filtering
  min_contour_size: 100
  max_contour_size: 10000
```

### Common Color Configurations

#### Black Objects
```yaml
h_lower: 0
h_upper: 179
s_lower: 0
s_upper: 255
v_lower: 0
v_upper: 50
```

#### Red Objects
```yaml
# Low red
h_lower: 0
h_upper: 10
s_lower: 100
s_upper: 255
v_lower: 50
v_upper: 255

# High red
h_lower: 170
h_upper: 179
s_lower: 100
s_upper: 255
v_lower: 50
v_upper: 255
```

#### Green Objects
```yaml
h_lower: 35
h_upper: 85
s_lower: 100
s_upper: 255
v_lower: 50
v_upper: 255
```

#### Blue Objects
```yaml
h_lower: 100
h_upper: 130
s_lower: 100
s_upper: 255
v_lower: 50
v_upper: 255
```

### Performance Optimization

#### Lighting Considerations
- **Avoid shadows**: Ensure even lighting
- **Minimize reflections**: Use matte surfaces
- **Consistent brightness**: Maintain stable lighting conditions

#### Camera Settings
- **Exposure**: Auto or manual adjustment
- **Focus**: Manual focus for consistent results
- **White balance**: Auto or fixed for stable colors

#### Parameter Tuning
1. Start with wide HSV ranges
2. Gradually narrow ranges for precision
3. Adjust contour size limits
4. Test in various lighting conditions

## AprilTag Tracking

### Understanding AprilTags
AprilTags are fiducial markers that provide:
- **Precise positioning**: Sub-pixel accuracy
- **Robust detection**: Works in various lighting
- **Unique identification**: Multiple tag support
- **Pose estimation**: 6DOF position and orientation

### Tag Setup

#### 1. Print Tags
Download and print AprilTags from [apriltag.org](https://apriltag.org/):
- **Tag Family**: tag36h11 (recommended)
- **Tag Size**: 16cm (adjustable)
- **Print Quality**: High resolution, no scaling

#### 2. Tag Placement
- **Flat surface**: Avoid curved or reflective surfaces
- **Good lighting**: Ensure even illumination
- **Camera view**: Position within camera field of view
- **Stable mounting**: Prevent movement during tracking

#### 3. Tag Configuration
Edit `config/apriltag_settings.yaml`:
```yaml
apriltag_ros:
  tag_family: 'tag36h11'
  tag_size: 0.16  # meters
  
  standalone_tags:
    - id: 0
      size: 0.16
      name: "target_tag"
```

### Configuration Parameters

#### Detection Parameters
```bash
# Set tag ID to track
rosparam set /apriltag_tracker/tag_id_to_track 0

# Set fallback position
rosparam set /apriltag_tracker/fallback_x 50.0
rosparam set /apriltag_tracker/fallback_y 50.0
rosparam set /apriltag_tracker/fallback_z 50.0

# Set failure threshold
rosparam set /apriltag_tracker/max_fail_count 100
```

#### Visualization Parameters
```bash
# Enable/disable visualization
rosparam set /apriltag_tracker/show_visualization true

# Set display scale
rosparam set /apriltag_tracker/resize_factor 0.5

# Set bounding box properties
rosparam set /apriltag_tracker/bbox_color "[0, 255, 0]"
rosparam set /apriltag_tracker/bbox_thickness 2
```

### Performance Optimization

#### Tag Detection
- **Tag size**: Larger tags are easier to detect
- **Distance**: Optimal detection range 0.5-3 meters
- **Lighting**: Avoid direct glare on tags
- **Camera angle**: Keep tags roughly perpendicular to camera

#### System Tuning
1. **Camera calibration**: Ensure accurate intrinsic parameters
2. **Tag placement**: Optimize for your use case
3. **Lighting**: Consistent, even illumination
4. **Camera settings**: Appropriate exposure and focus

## Advanced Configuration

### Home Zone Detection

The home zone feature monitors when tracked objects enter a defined area:

#### Configuration
```bash
# Set home zone boundaries
rosparam set /hsv_object_tracker/home_zone_x_min -30
rosparam set /hsv_object_tracker/home_zone_x_max 30
rosparam set /hsv_object_tracker/home_zone_z_min -50
rosparam set /apriltag_tracker/home_zone_z_max 50
```

#### Usage
```bash
# Monitor home zone events
rostopic echo /dog/home

# Custom home zone logic
rostopic echo /dog/position | grep -E "x: -?[0-9]+\.?[0-9]* z: -?[0-9]+\.?[0-9]*"
```

### Multi-Object Tracking

#### HSV Multi-Tracking
```python
# Modify hsv_tracker_node.py for multiple objects
def track_multiple_objects(self, hsv_image):
    # Find all contours above threshold
    contours = self._find_contours(hsv_image)
    
    # Sort by area (largest first)
    sorted_contours = sorted(contours, 
                            key=cv2.contourArea, 
                            reverse=True)
    
    # Return top N objects
    return sorted_contours[:3]
```

#### AprilTag Multi-Tracking
```yaml
# Configure multiple tags
apriltag_ros:
  standalone_tags:
    - id: 0
      size: 0.16
      name: "target_1"
    - id: 1
      size: 0.16
      name: "target_2"
    - id: 2
      size: 0.16
      name: "target_3"
```

### Custom Visualization

#### RViz Configuration
Create custom RViz configurations:
```yaml
# config/hsv_tracker.rviz
Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /dog/position1
      Splitter Ratio: 0.5
    Tree Height: 565
```

#### Custom Markers
```python
# Add custom visualization markers
def publish_tracking_marker(self, position, color):
    marker = Marker()
    marker.header.frame_id = "camera_color_optical_frame"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = position
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = color[0] / 255.0
    marker.color.g = color[1] / 255.0
    marker.color.b = color[2] / 255.0
    marker.color.a = 1.0
    
    self.marker_pub.publish(marker)
```

## Troubleshooting

### Common Issues

#### 1. Camera Not Detected
**Symptoms**: No camera topics, tracking fails
**Solutions**:
```bash
# Check camera connection
rs-enumerate-devices

# Verify USB connection
lsusb | grep RealSense

# Check camera topics
rostopic list | grep camera

# Restart camera node
rosnode kill /camera/realsense2_camera_node
roslaunch realsense2_camera rs_camera.launch
```

#### 2. Poor HSV Tracking
**Symptoms**: Unstable detection, false positives
**Solutions**:
- Adjust HSV parameters using sliders
- Check lighting conditions
- Verify camera focus
- Use morphological operations
- Adjust contour size limits

#### 3. AprilTag Not Detected
**Symptoms**: No tag detection, position not published
**Solutions**:
- Verify tag size configuration
- Check tag family selection
- Ensure proper lighting
- Verify camera calibration
- Check tag placement

#### 4. Depth Data Issues
**Symptoms**: Invalid depth values, tracking errors
**Solutions**:
- Clean camera lenses
- Check for reflective surfaces
- Verify depth camera alignment
- Adjust camera settings
- Check for IR interference

### Debug Mode

#### Enable Debug Logging
```bash
# Set ROS log level
export ROSCONSOLE_MIN_SEVERITY=DEBUG

# Or set in launch file
<param name="log_level" value="DEBUG"/>
```

#### Monitor System Status
```bash
# Check node status
rosnode list
rosnode info /hsv_object_tracker

# Monitor topics
rostopic hz /dog/position
rostopic echo /dog/position

# Check parameters
rosparam get /hsv_object_tracker
```

#### Performance Monitoring
```bash
# Monitor CPU usage
htop

# Check memory usage
free -h

# Monitor ROS performance
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_rect_raw
```

## API Reference

### ROS Topics

#### Published Topics
- **`/dog/position`** (`geometry_msgs/Point`)
  - **x**: X coordinate (pixels)
  - **y**: Y coordinate (pixels) 
  - **z**: Depth value (meters)

- **`/dog/home`** (`std_msgs/Empty`)
  - Published when object enters home zone

#### Subscribed Topics
- **`/camera/color/image_raw`** (`sensor_msgs/Image`)
  - Color camera stream

- **`/camera/depth/image_rect_raw`** (`sensor_msgs/Image`)
  - Depth camera stream

- **`/tag_detections`** (`apriltag_ros/AprilTagDetectionArray`)
  - AprilTag detection results

### ROS Parameters

#### HSV Tracker Parameters
```yaml
# HSV bounds
h_lower: int (0-179)
h_upper: int (0-179)
s_lower: int (0-255)
s_upper: int (0-255)
v_lower: int (0-255)
v_upper: int (0-255)

# Contour filtering
min_contour_size: int
max_contour_size: int

# Home zone
home_zone_x_min: int
home_zone_x_max: int
home_zone_z_min: int
home_zone_z_max: int
```

#### AprilTag Parameters
```yaml
# Fallback position
fallback_x: double
fallback_y: double
fallback_z: double

# Detection
max_fail_count: int
tag_id_to_track: int

# Visualization
show_visualization: bool
resize_factor: double
bbox_color: int array [3]
bbox_thickness: int
```

### Launch File Arguments

#### Common Arguments
```bash
# Enable/disable features
enable_hsv: bool
enable_apriltag: bool
rviz: bool

# HSV parameters
h_lower: int
h_upper: int
s_lower: int
s_upper: int
v_lower: int
v_upper: int

# AprilTag parameters
tag_id_to_track: int
fallback_x: double
fallback_y: double
fallback_z: double
```

### Example Usage

#### Basic HSV Tracking
```bash
roslaunch object_tracker hsv_tracker.launch
```

#### Custom HSV Parameters
```bash
roslaunch object_tracker hsv_tracker.launch \
    h_lower:=0 \
    h_upper:=50 \
    s_lower:=100 \
    s_upper:=255 \
    v_lower:=50 \
    v_upper:=255
```

#### Combined System
```bash
roslaunch object_tracker combined_tracker.launch \
    enable_hsv:=true \
    enable_apriltag:=true \
    h_lower:=0 \
    h_upper:=50 \
    tag_id_to_track:=0
```

---

For additional support, please refer to the main README or create an issue on GitHub.
