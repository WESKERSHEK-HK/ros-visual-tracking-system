cmake_minimum_required(VERSION 3.0.2)
project(ros_visual_tracking_system)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  realsense2_camera
  sensor_msgs
  std_msgs
  cv_bridge
  apriltag_ros
)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs 
    rospy 
    sensor_msgs 
    std_msgs
    realsense2_camera
    cv_bridge
    apriltag_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/ros_visual_tracking_system/nodes/hsv_tracker_node.py
  src/ros_visual_tracking_system/nodes/apriltag_tracker_node.py
  src/ros_visual_tracking_system/scripts/cv_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install documentation
install(FILES
  README.md
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
