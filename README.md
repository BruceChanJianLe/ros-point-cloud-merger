## Contents
- [HOW TO USE THIS PACKAGE](#how-to-use-this-package)
  - [Launch package](#launch-package)
  - [Launch package with particular configuration METHOD 1](#launch-package-with-particular-configuration-method-1)
  - [Launch package with particular configuration METHOD 2](#launch-package-with-particular-configuration-method-2)
  - [Launch package with maximum and minimum range enabled/disabled METHOD 1](#launch-package-with-maximum-and-minimum-range-enableddisabled-method-1)
  - [Launch package with maximum and minimum range enabled/disabled METHOD 2](#launch-package-with-maximum-and-minimum-range-enableddisabled-method-2)
- [HOW TO CHANGE CONFIGURATION](#how-to-change-configuration)
  - [Change input topics in husky.yaml](#change-input-topics-in-huskyyaml)
  - [Change output topic in husky.yaml](#change-output-topic-in-huskyyaml)
  - [Change output frame ID in husky.yaml](#change-output-frame-id-in-huskyyaml)
  - [Change minimum range in x axis in husky_x.yaml](#change-minimum-range-in-x-axis-in-husky_xyaml)
  - [Change minimum range in y axis in husky_y.yaml](#change-minimum-range-in-y-axis-in-husky_yyaml)
  - [Change maximum range in x axis in husky_x.yaml](#change-maximum-range-in-x-axis-in-husky_xyaml)
  - [Change maximum range in y axis in husky_y.yaml](#change-maximum-range-in-y-axis-in-husky_yyaml)
  - [Change minimum height in z axis in husky_z.yaml](#change-minimum-height-in-z-axis-in-husky_zyaml)
  - [Change maximum height in z axis in husky_z.yaml](#change-maximum-height-in-z-axis-in-husky_zyaml)
- [WHAT EXISTING FILES TO CHANGE](#what-existing-files-to-change)
  - [What to add in to robot urdf](#what-to-add-in-to-robot-urdf)
  - [How package.xml should look like](#how-packagexml-should-look-like)
  - [How CMakeLists.txt should look like](#how-cmakeliststxt-should-look-like)
- [NEW FILES TO ADD IN](#new-files-to-add-in)
  - [New file point_cloud_merger.launch](#new-file-point_cloud_mergerlaunch)
  - [New file point_cloud_merger_node.cpp](#new-file-point_cloud_merger_nodecpp)
  - [New file point_cloud_merger.hpp](#new-file-point_cloud_mergerhpp)
  - [New file point_cloud_merger.cpp](#new-file-point_cloud_mergercpp)
  - [New file test_launch.test](#new-file-test_launchtest)
  - [New file test.cpp](#new-file-testcpp)
- [HOW TO RUN TESTING](#how-to-run-testing)
  - [Testing google tests](#testing-google-tests)
  - [Running rosbag](#running-rosbag)
- [EXAMPLE IMPLEMENTATION](#example-implementation)
- [REFERENCES](#references)

<br>

# HOW TO USE THIS PACKAGE

## Launch package

```
roslaunch ros-point-cloud-merger point_cloud_merger.launch
```

>By default, configuration would be 
- ```husky_x.yaml``` for x axis range
- ```husky_y.yaml``` for y axis range
- ```husky_z.yaml``` for z axis range
- ```husky.yaml``` for input topics, output topic and output frame ID

>Configured files can be accessed in [here](https://github.com/BruceChanJianLe/ros-point-cloud-merger/blob/branch-merge/config)

<br>

## Launch package with particular configuration METHOD 1

in **terminal**
```
roslaunch ros-point-cloud-merger point_cloud_merger.launch robot_name:=husky x_range:=husky_x y_range:=husky_y z_range:=husky_z
```
>this will launch package with [husky](https://github.com/BruceChanJianLe/ros-point-cloud-merger/blob/branch-merge/config/husky.yaml) configuration

```
roslaunch ros-point-cloud-merger point_cloud_merger.launch robot_name:=kobuki x_range:=kobuki_x y_range:=kobuki_y z_range:=kobuki_z
```
>this will launch package with [kobuki](https://github.com/BruceChanJianLe/ros-point-cloud-merger/blob/branch-merge/config/kobuki.yaml) configuration

<br>

## Launch package with particular configuration METHOD 2

in **point_cloud_merger.launch**
```
<arg name="robot_name" default="husky" />
<arg name="x_range" default="husky_x" />
<arg name="y_range" default="husky_y" />
<arg name="z_range" default="husky_z" />
```

>in this case, this takes in params from ```husky.yaml```, ```husky_x.yaml```, ```husky_y.yaml```, and ```husky_z.yaml```.

<br>

## Launch package with maximum and minimum range enabled/disabled METHOD 1

in **terminal**
```
roslaunch ros-point-cloud-merger point_cloud_merger.launch enable_range_flag:=true
```
>this will launch package with maximum and minimum range enabled

```
roslaunch ros-point-cloud-merger point_cloud_merger.launch enable_range_flag:=false
```
>this will launch package with maximum and minimum range disabled

<br>

## Launch package with maximum and minimum range enabled/disabled METHOD 2

in **point_cloud_merger.launch**
```
<arg name="enable_range_flag" default="true"/>
```
>this will launch package with maximum and minimum range enabled

```
<arg name="enable_range_flag" default="false"/>
```
>this will launch package with maximum and minimum range disabled

<br>

# HOW TO CHANGE CONFIGURATION

**configurations for each robot can be found in config folder in [this](https://github.com/BruceChanJianLe/ros-point-cloud-merger) package**

- Currently there is only configurations for 2 robots, *husky* and *kobuki*
- With each robot configuration being modifiable
- Adding new configuration files is available too!

## Change input topics in husky.yaml

in this case, **/alpha** and **/beta** are the pointclouds that you want to merge
```
input_topics: "[/alpha, /beta]"
```

## Change output topic in husky.yaml

in this case, **/husky_points_concat** is output topic name for the merged pointcloud
```
output_topic: '/husky_points_concat'
```

## Change output frame ID in husky.yaml

in this case, **velodyne_frame** is output frame id for the merged pointcloud
```
output_frame_id: 'velodyne_frame'
```

**when modifying the value here, have to also modify at the robot urdf**

## Change minimum range in x axis in husky_x.yaml

in this case, **0.5** and **-0.5** are desired minimum range in the positive x axis and negative x axis respectively
```
pmin_range_x: 0.5
nmin_range_x: -0.5
```

## Change minimum range in y axis in husky_y.yaml

in this case, **0.5** and **-0.5** are desired minimum range in the positive y axis and negative y axis respectively
```
pmin_range_y: 0.5
nmin_range_y: -0.5
```

## Change maximum range in x axis in husky_x.yaml

in this case, **5.0** and **-5.0** are desired maximum range in the positive x axis and negative x axis respectively
```
pmax_range_x: 5.0
nmax_range_x: -5.0
```

## Change maximum range in y axis in husky_y.yaml

in this case, **5.0** and **-5.0** are desired maximum range in the positive y axis and negative y axis respectively
```
pmax_range_y: 5.0
nmax_range_y: -5.0
```
## Change minimum height in z axis in husky_z.yaml

in this case, **-1.0** is desired minimum height in the z axis 
```
pmin_range_z: -1.0
```

## Change maximum height in z axis in husky_z.yaml

in this case, **100.0** is desired maximum height in the z axis
```
pmax_range_z: 100.0
```

<br>

# WHAT EXISTING FILES TO CHANGE

## What to add in to robot urdf

in this case, the output fame id is velodyne_frame

```
  <xacro:arg name="output_frame_id" default="$(optenv OUTPUT_FRAME_ID velodyne_frame)" />

  <!-- Concatenate point clouds -->
  <link name="$(arg output_frame_id)" />
  <joint name="velodyne_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="$(arg output_frame_id)" />
  </joint>
```

>An example of implementation [here](https://github.com/e0425705/husky_sw/blob/branch-package/husky_description/urdf/husky.urdf.xacro)

## How package.xml should look like

```
<?xml version="1.0"?>
<package format="2">
  <name>ros-point-cloud-merger</name>
  <version>0.1.0</version>
  <description>The ros-point-cloud-merger package</description>

  <author email="jianle001@e.ntu.edu.sg">Bruce Chan Jian Le</author>
  <maintainer email="jianle001@e.ntu.edu.sg">Bruce Chan Jian Le</maintainer>
  <maintainer email="e0425705@u.nus.edu">Puah Siew Wen</maintainer>
  <license>MIT</license>
  <license>BSD</license>
  <url type="website">https://github.com/BruceChanJianLe/ros-point-cloud-merger</url>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>geometry_msgs</build_depend>
  <build_depend>message_filters</build_depend>
  <build_depend>pcl_conversions</build_depend>
  <build_depend>pcl_ros</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>tf2</build_depend>
  <build_depend>tf2_ros</build_depend>

  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>message_filters</build_export_depend>
  <build_export_depend>pcl_conversions</build_export_depend>
  <build_export_depend>pcl_ros</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>tf2</build_export_depend>
  <build_export_depend>tf2_ros</build_export_depend>

  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_filters</exec_depend>
  <exec_depend>pcl_conversions</exec_depend>
  <exec_depend>pcl_ros</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>tf2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <test_depend>rostest</test_depend>

  <export>
  </export>
</package>
```

## How CMakeLists.txt should look like

```
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
  add_rostest_gtest(test_mynode test/test_launch.test test/test.cpp src/point_cloud_merger.cpp)
  target_link_libraries(test_mynode ${catkin_LIBRARIES})
endif()

install(
  TARGETS
    point_cloud_merger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

<br>

# NEW FILES TO ADD IN

## New file point_cloud_merger.launch

this is the launch file of point_cloud_merger

## New file point_cloud_merger_node.cpp

this is the cpp file that launches the point_cloud_merger node

## New file point_cloud_merger.hpp

this is the hpp file for point_cloud_merger

## New file point_cloud_merger.cpp

this is the cpp file for point_cloud_merger

## New file test_launch.test

this is the test file 

## New file test.cpp

this is the file that contains the tests

<br>

# HOW TO RUN TESTING

## Testing google tests

terminal 1:

```
catkin_make run_tests_ros-point-cloud-merger
```

OR

```
rostest ros-point-cloud-merger test_launch.test
```

## Running rosbag

terminal 1

```
roscore
```

terminal 2

```
rosbag play 8pts.bag -l
```

```
rosbag play 8pts.bag -l -r 10
```

- speed X10

get fixed frame for rviz

```
rostopic echo /kobuki_points_colncat | grep frame_id
```

terminal 3
- get the point clouds data

```
rostopic echo /kobuki_points_concat
```

terminal 4
- launch rviz

```
rviz
```

- ```Fixed Frame``` == ```rostopic echo /kobuki_points_colncat | grep frame_id```
- ```PointCloud2``` ```Topic``` == ```husky_points_concat```
- topic name varies depending on what topics are recorded in rosbag 

<br>

# EXAMPLE IMPLEMENTATION
- [example - package](https://github.com/e0425705/husky_sw/tree/branch-package)
- [example - shared pointer](https://github.com/BruceChanJianLe/ros-point-cloud-merger/tree/branch-shared-pointer)
- [example - test](https://github.com/BruceChanJianLe/ros-point-cloud-merger/tree/branch-unit-testing)

<br>

# REFERENCES
- [Core Perception - launch file](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/launch/points_concat_filter.launch)
- [Core Perception - cpp](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/nodes/points_concat_filter/points_concat_filter.cpp)