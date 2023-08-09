cmake_minimum_required(VERSION 3.0.2)
project(ransac_gpd)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

find_package(PCL 1.8 REQUIRED)

find_package(catkin REQUIRED genmsg actionlib_msgs roscpp pcl_ros)

add_action_files(
  DIRECTORY action
  FILES get_grasping_point.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(listener src/listener.cpp)
add_executable(viewer src/viewer.cpp)

add_dependencies(viewer ${PROJECT_NAME}_gencfg)
add_dependencies(listener ${PROJECT_NAME}_gencfg)

target_link_libraries(listener ${catkin_LIBRARIES})
target_link_libraries(viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})