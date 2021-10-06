## Contents
- [HOW TO USE THIS PACKAGE](#how-to-use-this-package)
  - [Launch package](#launch-package)
- [HOW TO CHANGE CONFIGURATION](#how-to-change-configuration)
  - [Change input topics](#change-input-topics)
  - [Change output topic](#change-output-topic)
  - [Change output frame id](#change-output-frame-id)
  - [Change minimum range x axis](#change-minimum-range-x-axis)
  - [Change minimum range y axis](#change-minimum-range-y-axis)
  - [Change maximum range x axis](#change-maximum-range-x-axis)
  - [Change maximum range y axis](#change-maximum-range-y-axis)
  - [Change minimum height z axis](#change-minimum-height-z-axis)
  - [Change maximum height z axis](#change-maximum-height-z-axis)
- [WHAT EXISTING FILES TO CHANGE](#what-existing-files-to-change)
  - [What to add in to robot urdf](#what-to-add-in-to-robot-urdf)
  - [How package.xml should look like](#how-packagexml-should-look-like)
  - [How CMakeLists.txt should look like](#how-cmakeliststxt-should-look-like)
- [NEW FILES TO ADD IN](#new-files-to-add-in)
  - [New file point_cloud_merger.launch](#new-file-point_cloud_mergerlaunch)
  - [New file point_cloud_merger_node.cpp](#new-file-point_cloud_merger_nodecpp)
  - [New file point_cloud_merger.hpp](#new-file-point_cloud_mergerhpp)
  - [New file point_cloud_merger.cpp](#new-file-point_cloud_mergercpp)
- [EXAMPLE IMPLEMENTATION](#example-implementation)
- [REFERENCES](#references)

<br>

# HOW TO USE THIS PACKAGE

## Launch package

```
roslaunch ros-point-cloud-merger point_cloud_merger.launch
```
>By default, configuration would be husky.yaml

>Configuration can be accessed in [here](https://github.com/BruceChanJianLe/ros-point-cloud-merger/blob/branch-merge/config/husky.yaml)]

<br>

# HOW TO CHANGE CONFIGURATION

**configurations for each robot can be found in config folder in [this](https://github.com/BruceChanJianLe/ros-point-cloud-merger) package**

- Currently there is only configurations for 2 robots, *husky* and *kobuki*
- With each robot configuration being modifiable
- Adding new configuration files is available too!

**the following change in configuration will be done on husky.yaml**

## Change input topics

in this case, **/alpha** and **/beta** are the pointclouds that you want to merge
```
input_topics: "[/alpha, /beta]"
```

## Change output topic

in this case, **/husky_points_concat** is output topic name for the merged pointcloud
```
output_topic: '/husky_points_concat'
```

## Change output frame id

in this case, **velodyne_frame** is output frame id for the merged pointcloud
```
output_frame_id: 'velodyne_frame'
```

**when modifying the value here, have to also modify at the robot urdf**

## Change minimum range x axis

in this case, **0.5** and **-0.5** are desired minimum range in the positive x axis and negative x axis respectively
```
pmin_range_x: 0.5
nmin_range_x: -0.5
```

## Change minimum range y axis

in this case, **0.5** and **-0.5** are desired minimum range in the positive y axis and negative y axis respectively
```
pmin_range_y: 0.5
nmin_range_y: -0.5
```

## Change maximum range x axis

in this case, **5.0** and **-5.0** are desired maximum range in the positive x axis and negative x axis respectively
```
pmax_range_x: 5.0
nmax_range_x: -5.0
```

## Change maximum range y axis

in this case, **5.0** and **-5.0** are desired maximum range in the positive y axis and negative y axis respectively
```
pmax_range_y: 5.0
nmax_range_y: -5.0
```
## Change minimum height z axis

in this case, **-1.0** is desired minimum height in the z axis 
```
pmin_range_z: -1.0
```

## Change maximum height z axis

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

## How CMakeLists.txt should look like

<br>

# NEW FILES TO ADD IN

## New file point_cloud_merger.launch

## New file point_cloud_merger_node.cpp

## New file point_cloud_merger.hpp

## New file point_cloud_merger.cpp

<br>

# EXAMPLE IMPLEMENTATION
[example](https://github.com/e0425705/husky_sw/tree/branch-package)

<br>

# REFERENCES
- [Core Perception - launch file](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/launch/points_concat_filter.launch)
- [Core Perception - cpp](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/nodes/points_concat_filter/points_concat_filter.cpp)