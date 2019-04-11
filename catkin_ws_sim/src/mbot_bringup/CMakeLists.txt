cmake_minimum_required(VERSION 2.8.3)
project(mbot_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mbot_bringup
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(mbot_bringup src/mbot_bringup.cpp src/mbot.cpp)
target_link_libraries(mbot_bringup ${catkin_LIBRARIES})












