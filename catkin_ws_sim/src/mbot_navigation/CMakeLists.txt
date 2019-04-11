cmake_minimum_required(VERSION 2.8.3)
project(mbot_navigation)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbot_navigation
#  CATKIN_DEPENDS geometry_msgs move_base_msgs roscpp rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

