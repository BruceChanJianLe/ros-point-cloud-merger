Content
- [Naming for file](#naming-for-file)
- [Generate  world file from building map](#generate--world-file-from-building-map)
- [Download models used in newly created traffic editor building map](#download-models-used-in-newly-created-traffic-editor-building-map)
- [Generate Traffic Navigation Path File](#generate-traffic-navigation-path-file)
- [Display in Gazebo](#display-in-gazebo)
- [Example](#example)
- [Reference](#reference)

# Naming for file

```
*.building.yaml
```

# Generate  world file from building map

Template
```
ros2 run rmf_building_map_tools building_map_generator gazebo \  ${building_map_path} ${output_world_path} $(output_model_dir}
```

Example
```
ros2 run rmf_building_map_tools building_map_generator gazebo \
  /home/isera2/catkin_ws/src/ros-point-cloud-merger/yaml/level9_north_south.building.yaml /home/isera2/catkin_ws/src/ros-point-cloud-merger/worlds/level9_north_south.world /home/isera2/catkin_ws/src/ros-point-cloud-merger/models
```

# Download models used in newly created traffic editor building map

Template
```
ros2 run rmf_building_map_tools building_map_model_downloader \
  ${building_map_path} -f -e ~/.gazebo/models
```

Example
```
ros2 run rmf_building_map_tools building_map_model_downloader \
  /home/isera2/catkin_ws/src/ros-point-cloud-merger/yaml/level9_north_south.building.yaml -f -e ~/.gazebo/models
```

# Generate Traffic Navigation Path File

Template
```
ros2 run rmf_building_map_tools building_map_generator nav \
  ${building_map_path} ${output_nav_graphs_dir}
```

# Display in Gazebo

```
gazebo /home/isera2/catkin_ws/src/ros-point-cloud-merger/worlds/level9_north_south.world
```

Template
```
gazebo worlds/<filename>
```

Example
```
gazebo worlds/level9_north_south.world
```

launch file
```
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/mud.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="recording" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
```

# Example

```
ros2 run rmf_building_map_tools building_map_generator gazebo   /home/isera2/catkin_
ws/src/ros-point-cloud-merger/yaml/I2R_LVL9.building.yaml /home/isera2/catkin_ws/src/ros-point-cloud-merger/worlds/I2R_LVL9.world /home/isera2/catkin_ws/src/ros-point-cloud-merger/models
```

```
ros2 run rmf_building_map_tools building_map_model_downloader   /home/isera2/catkin_ws/src/ros-point-cloud-merger/yaml/I2R_LVL9.building.yaml -f -e ~/.gazebo/models
```

```
gazebo /home/isera2/catkin_ws/src/ros-point-cloud-merger/worlds/I2R_LVL9.world
```

# Reference

- [GitHub](https://github.com/open-rmf/rmf_traffic_editor)
