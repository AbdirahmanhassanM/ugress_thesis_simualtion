cmake_minimum_required(VERSION 3.0.2)
project(ugress_sim)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_ros
)

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs
    nav_msgs
    gazebo_ros
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/follow_path_final.py
  scripts/route_square.py
  scripts/route_zigzag.py
  scripts/route_spiral.py
  scripts/route_circle.py
  scripts/select_route.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

## Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE
)

## Install world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE
)