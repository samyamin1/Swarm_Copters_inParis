cmake_minimum_required(VERSION 3.0.2)
project(paris_swarm_simulation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  gazebo_ros
  gazebo_plugins
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_ros
    tf2_geometry_msgs
    gazebo_ros
    gazebo_plugins
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
install(PROGRAMS
  scripts/communication_logger.py
  scripts/performance_monitor.py
  ai_controllers/flight_controller.py
  ai_controllers/swarm_coordinator.py
  control_panel/mission_control.py
  quadcopter_models/quadcopter_urdf.py
  paris_environment/paris_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.rviz"
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
)

# Install model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.sdf"
)

# Install documentation
install(DIRECTORY docs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
  FILES_MATCHING PATTERN "*.md" PATTERN "*.rst"
) 