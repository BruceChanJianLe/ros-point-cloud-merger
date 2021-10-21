cmake_minimum_required(VERSION 3.0.2)
project(ros-point-cloud-merger)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

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

find_package(rostest REQUIRED)

find_package(PkgConfig REQUIRED)

find_package(PCL REQUIRED)

find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)

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
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
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
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(point_cloud_merger_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

## will cause the executable to be built during the main build (a simple make), and will put it in TBD
#catkin_add_gtest(${PROJECT_NAME} 
#  test/test_launch.test
#  test/test.cpp
#)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_mynode test/test_launch.test test/test.cpp)
  target_link_libraries(test_mynode ${catkin_LIBRARIES})
endif()

install(
  TARGETS
    point_cloud_merger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)