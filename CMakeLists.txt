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
  sensor_msgs
  tf
)

find_package(PkgConfig REQUIRED)

find_package(PCL REQUIRED)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)

find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros-point-cloud-merger
#  CATKIN_DEPENDS geometry_msgs message_filters pcl_conversions pcl_ros roscpp rospy sensor_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
)

link_directories(
  ${YAML_CPP_LIBRARY_DIRS}
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
  ${YAML_CPP_LIBRARIES}
)

install(
  TARGETS
    point_cloud_merger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)