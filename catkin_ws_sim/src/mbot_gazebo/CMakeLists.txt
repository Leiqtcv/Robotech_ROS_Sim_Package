cmake_minimum_required(VERSION 2.8.3)
project(mbot_gazebo)


find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbot_gazebo
#  CATKIN_DEPENDS gazebo_plugins gazebo_ros gazebo_ros_control roscpp rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
