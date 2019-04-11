cmake_minimum_required(VERSION 2.8.3)
project(mbot_description)


find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbot_description
#  CATKIN_DEPENDS urdf xacro
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


