cmake_minimum_required(VERSION 3.0.2)
project(ransac_gpd)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})