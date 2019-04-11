cmake_minimum_required(VERSION 2.8.3)
project(mbot_sensor)

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
)


catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES mbot_sensor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_filter src/lidar_filter.cpp src/LinkedList.cpp)
target_link_libraries(lidar_filter ${catkin_LIBRARIES})

add_executable(LinkedList_Demo src/LinkedList.cpp src/LinkedList_Demo.cpp)
target_link_libraries(LinkedList_Demo ${catkin_LIBRARIES})
