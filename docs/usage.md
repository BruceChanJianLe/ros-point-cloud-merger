Content
- [Description](#description)
- [Download](#download)
- [Launch](#launch)
- [Type for editable file in traffic editor](#type-for-editable-file-in-traffic-editor)
- [Generate world file from building map](#generate-world-file-from-building-map)
- [Download models used in newly created traffic editor building map](#download-models-used-in-newly-created-traffic-editor-building-map)
- [Generate Traffic Navigation Path File](#generate-traffic-navigation-path-file)
- [Display in Gazebo](#display-in-gazebo)
- [Example](#example)
- [Reference](#reference)

# Description
- this shows how to convert from *.building.yaml in traffic editor to *.world to be displayed in Gazebo

# Download

Follow instructions located [here](https://github.com/open-rmf/rmf_traffic_editor) to install traffic editor

# Launch

To launch traffic editor

```
traffic-editor
```

# Type for editable file in traffic editor
```
*.building.yaml
```

Example
```
LVL9.building.yaml
```

# Generate world file from building map

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

Template
```
gazebo ${output_world_path}
```

Example
```
gazebo /home/isera2/catkin_ws/src/ros-point-cloud-merger/worlds/level9_north_south.world
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
