Contents
- [RUN TESTING](#run-testing)
  - [Testing tests](#testing-tests)
  - [Files](#files)
    - [New file test_launch.test](#new-file-test_launchtest)
    - [New file test.cpp](#new-file-testcpp)
  - [Running rosbag](#running-rosbag)

# RUN TESTING

## Testing tests

terminal 1:

```
catkin_make run_tests_ros-point-cloud-merger
```

## Files

### New file test_launch.test

this is the test file 

### New file test.cpp

this is the file that contains the tests

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

- ```Fixed Frame``` == ```rostopic echo /kobuki_points_concat | grep frame_id```
- ```PointCloud2``` ```Topic``` == ```husky_points_concat```
- topic name varies depending on what topics are recorded in rosbag 

<br>