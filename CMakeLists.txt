cmake_minimum_required(VERSION 3.0.2)
project(ros-point-cloud-merger)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)

find_package(PkgConfig REQUIRED)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros-point-cloud-merger
#  CATKIN_DEPENDS geometry_msgs message_filters pcl_conversions pcl_ros roscpp rospy sensor_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(point_cloud_merger_node
  src/point_cloud_merger_node.cpp
  src/point_cloud_merger.cpp
)

add_dependencies(point_cloud_merger_node
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(point_cloud_merger_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(point_cloud_merger_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME} src/test.cpp)

install(
  TARGETS
    point_cloud_merger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)