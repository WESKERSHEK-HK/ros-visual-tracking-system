# ROS Visual Tracking System

[![ROS Version](https://img.shields.io/badge/ROS-Melodic-brightgreen.svg)](https://www.ros.org/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.6+-blue.svg)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-blue.svg)](https://opencv.org/)

A professional ROS package for real-time visual object tracking using computer vision with support for both HSV color-based tracking and AprilTag fiducial marker detection. Integrates seamlessly with RealSense cameras for depth-aware object positioning.

## 🚀 Features

- **Dual Tracking Methods**: HSV color-based tracking and AprilTag detection
- **RealSense Integration**: Native support for Intel RealSense D400 series cameras
- **Depth Awareness**: 3D positioning with depth camera integration
- **Real-time Performance**: Optimized for low-latency tracking applications
- **Configurable Parameters**: Extensive parameter tuning via ROS parameters
- **Professional Architecture**: Clean, modular code structure with proper error handling
- **Visualization Tools**: Built-in tracking visualization and debugging tools
- **Home Zone Detection**: Configurable home zone monitoring for robotic applications

## 📋 Requirements

### System Requirements
- **ROS**: Melodic (Ubuntu 18.04) or Noetic (Ubuntu 20.04)
- **Python**: 3.6 or higher
- **OpenCV**: 4.x
- **Camera**: Intel RealSense D400 series (D435i recommended)

### ROS Dependencies
```bash
sudo apt-get install ros-melodic-realsense2-camera
sudo apt-get install ros-melodic-apriltag-ros
sudo apt-get install ros-melodic-cv-bridge
sudo apt-get install ros-melodic-geometry-msgs
sudo apt-get install ros-melodic-sensor-msgs
sudo apt-get install ros-melodic-std-msgs
```

## 🏗️ Installation

### 1. Clone the Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/ros-visual-tracking-system.git
```

### 2. Build the Package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Install Python Dependencies
```bash
pip3 install numpy opencv-python
```

## 🎯 Quick Start

### HSV Color-based Tracking
```bash
# Launch HSV tracker with RealSense camera
roslaunch ros_visual_tracking_system hsv_tracker.launch

# Launch with RViz visualization
roslaunch ros_visual_tracking_system hsv_tracker.launch rviz:=true
```

### AprilTag Tracking
```bash
# Launch AprilTag tracker with RealSense camera
roslaunch ros_visual_tracking_system apriltag_tracker.launch

# Launch with RViz visualization
roslaunch ros_visual_tracking_system apriltag_tracker.launch rviz:=true
```

### Combined Tracking System
```bash
# Launch both tracking systems
roslaunch ros_visual_tracking_system combined_tracker.launch

# Launch with custom parameters
roslaunch ros_visual_tracking_system combined_tracker.launch \
    enable_hsv:=true \
    enable_apriltag:=true \
    h_lower:=0 \
    h_upper:=50 \
    tag_id_to_track:=0
```

## 📖 Usage Guide

### HSV Tracker Configuration

The HSV tracker uses color-based object detection. Adjust the HSV parameters using the interactive sliders or ROS parameters:

```bash
# Set HSV bounds via ROS parameters
rosparam set /hsv_object_tracker/h_lower 0
rosparam set /hsv_object_tracker/h_upper 179
rosparam set /hsv_object_tracker/s_lower 0
rosparam set /hsv_object_tracker/s_upper 255
rosparam set /hsv_object_tracker/v_lower 0
rosparam set /hsv_object_tracker/v_upper 50
```

**Common HSV Values:**
- **Black**: H: 0-179, S: 0-255, V: 0-50
- **Red**: H: 0-10 or 170-179, S: 100-255, V: 50-255
- **Green**: H: 35-85, S: 100-255, V: 50-255
- **Blue**: H: 100-130, S: 100-255, V: 50-255

### AprilTag Configuration

Configure AprilTag detection in `config/apriltag_settings.yaml`:

```yaml
apriltag_ros:
  tag_family: 'tag36h11'
  tag_size: 0.16
  standalone_tags:
    - id: 0
      size: 0.16
      name: "target_tag"
```

### Home Zone Detection

Configure home zone boundaries for robotic applications:

```bash
# Set home zone parameters
rosparam set /hsv_object_tracker/home_zone_x_min -30
rosparam set /hsv_object_tracker/home_zone_x_max 30
rosparam set /hsv_object_tracker/home_zone_z_min -50
rosparam set /hsv_object_tracker/home_zone_z_max 50
```

## 📡 ROS Topics

### Published Topics
- `/dog/position` (`geometry_msgs/Point`): Object position in 3D space
- `/dog/home` (`std_msgs/Empty`): Home zone detection signal

### Subscribed Topics
- `/camera/color/image_raw` (`sensor_msgs/Image`): Color camera stream
- `/camera/depth/image_rect_raw` (`sensor_msgs/Image`): Depth camera stream
- `/tag_detections` (`apriltag_ros/AprilTagDetectionArray`): AprilTag detections

## ⚙️ Configuration

### HSV Tracker Parameters
Edit `config/hsv_tracker_params.yaml` for persistent HSV tracking configuration:

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
  
  # Home zone
  home_zone_x_min: -30
  home_zone_x_max: 30
  home_zone_z_min: -50
  home_zone_z_max: 50
```

### AprilTag Parameters
Edit `config/apriltag_settings.yaml` for AprilTag detection configuration:

```yaml
apriltag_ros:
  tag_family: 'tag36h11'
  tag_size: 0.16
  standalone_tags:
    - id: 0
      size: 0.16
      name: "target_tag"
```

## 🧪 Testing

### Test HSV Tracking
```bash
# Launch HSV tracker
roslaunch object_tracker hsv_tracker.launch

# In another terminal, monitor position data
rostopic echo /dog/position

# Monitor home zone detection
rostopic echo /dog/home
```

### Test AprilTag Tracking
```bash
# Launch AprilTag tracker
roslaunch object_tracker apriltag_tracker.launch

# In another terminal, monitor position data
rostopic echo /dog/position
```

### Test with Test Images
```bash
# Use the provided test scripts
rosrun object_tracker cv_test.py
rosrun object_tracker display_image.py
```

## 🔧 Troubleshooting

### Common Issues

1. **Camera Not Detected**
   ```bash
   # Check RealSense camera connection
   rs-enumerate-devices
   
   # Verify camera topics
   rostopic list | grep camera
   ```

2. **Poor Tracking Performance**
   - Adjust HSV parameters using the interactive sliders
   - Check lighting conditions
   - Verify camera focus and exposure

3. **AprilTag Not Detected**
   - Ensure proper tag size configuration
   - Check tag family selection
   - Verify camera calibration

4. **Depth Data Issues**
   - Clean camera lenses
   - Check for reflective surfaces
   - Verify depth camera alignment

### Debug Mode
Enable debug logging for detailed information:

```bash
# Set ROS log level
export ROSCONSOLE_MIN_SEVERITY=DEBUG

# Or set in launch file
<param name="log_level" value="DEBUG"/>
```

## 📁 Project Structure

```
ros-visual-tracking-system/
├── src/ros_visual_tracking_system/
│   ├── nodes/                 # Main tracking nodes
│   │   ├── hsv_tracker_node.py
│   │   └── apriltag_tracker_node.py
│   ├── utils/                 # Utility modules
│   │   ├── tracking_utils.py
│   │   └── visualization.py
│   ├── launch/                # Launch files
│   │   ├── hsv_tracker.launch
│   │   ├── apriltag_tracker.launch
│   │   └── combined_tracker.launch
│   ├── config/                # Configuration files
│   │   ├── apriltag_settings.yaml
│   │   └── hsv_tracker_params.yaml
│   └── scripts/               # Utility scripts
│       ├── cv_test.py
│       └── display_image.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🤝 Contributing

We welcome contributions! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Setup
```bash
# Install development dependencies
pip3 install -r requirements-dev.txt

# Run tests
python3 -m pytest tests/

# Check code style
flake8 src/
black src/
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Intel RealSense** for camera integration
- **AprilTag** for fiducial marker detection
- **OpenCV** for computer vision capabilities
- **ROS Community** for the robotics framework

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/yourusername/ros-visual-tracking-system/issues)
- **Discussions**: [GitHub Discussions](https://github.com/yourusername/ros-visual-tracking-system/discussions)
- **Wiki**: [Project Wiki](https://github.com/yourusername/ros-visual-tracking-system/wiki)

## 🔄 Changelog

### Version 1.0.0
- Initial professional release
- HSV color-based tracking
- AprilTag detection
- RealSense camera integration
- Modular architecture
- Comprehensive documentation

---

**Made with ❤️ by the Object Tracker Team**
