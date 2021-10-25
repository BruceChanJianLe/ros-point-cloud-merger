Content
- [Display ```rosbag play``` on rviz](#display-rosbag-play-on-rviz)

## Display ```rosbag play``` on rviz

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