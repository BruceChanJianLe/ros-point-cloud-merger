# [GitHub Pages](https://e0425705.github.io/husky_sw/)

## Contents
- [GitHub Pages](#github-pages)
  - [Contents](#contents)
- [HOW TO USE THIS PACKAGE](#how-to-use-this-package)
- [WHAT EXISTING FILES TO CHANGE](#what-existing-files-to-change)
  - [Add in xml-package](#add-in-xml-package)
  - [Add in txt-CMakeLists](#add-in-txt-cmakelists)
- [NEW FILES TO ADD IN](#new-files-to-add-in)
  - [new file cpp-points_concat_filter_node](#new-file-cpp-points_concat_filter_node)
  - [new file launch-points_concat_filter](#new-file-launch-points_concat_filter)
  - [new file hpp-points_concat_filter](#new-file-hpp-points_concat_filter)
  - [new file cpp-points_concat_filter](#new-file-cpp-points_concat_filter)
- [ADDITIONAL NOTES](#additional-notes)
- [REFERENCES](#references)
- [GitHub Pages](#github-pages-1)

<br>

# HOW TO USE THIS PACKAGE

```
roslaunch ros-point-cloud-merger points_concat_filter.launch
```

<br>

# WHAT EXISTING FILES TO CHANGE

## Add in xml-package
- package.xml

```
<build_depend>yaml-cpp</build_depend>

<build_export_depend>yaml-cpp</build_export_depend>

<exec_depend>yaml-cpp</exec_depend>
```

## Add in txt-CMakeLists
- CMakeLists.txt

```
find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)

find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)

add_executable(points_concat_filter_node
  src/points_concat_filter_node.cpp
  src/points_concat_filter.cpp
)

add_dependencies(points_concat_filter_node ${catkin_EXPORTED_TARGETS})

target_include_directories(points_concat_filter_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(points_concat_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(
  TARGETS
    points_concat_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

<br>

# NEW FILES TO ADD IN

## new file cpp-points_concat_filter_node
- points_concat_filter_node.cpp

```
#include "ros-point-cloud-merger/points_concat_filter.hpp"
#include <ros/ros.h>

const std::string ROSNodeName = "points_concat_filter";

// launch node for points_concat_filter.cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, ROSNodeName);

    PointsConcatFilter node;

    ros::spin();

    return 0;
}
```

## new file launch-points_concat_filter
- points_concat_filter.launch

```
<?xml version="1.0"?>

<!-- type is the file you want to launch in the package 
  name is a unique identifier for your node. -->
<launch>

  <!-- replace the topics with the ones you want as input -->
  <arg name="input_topics" default="[/velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]" />
  <!-- replace with the desired output topic -->
  <arg name="output_topic" default="/points_concat" />
  <!-- replace with desired output frame -->
  <arg name="output_frame_id" default="velodyne_frame" />

  <!-- replace with desired min range from base_link -->
  <!-- <arg name="min_range" default="0.9" /> -->
  <!-- replace with desired max range from base_link -->
  <!-- <arg name="max_range" default="2.0" /> -->

  <node name="points_concat_filter_node" output="screen" pkg="ros-point-cloud-merger" type="points_concat_filter_node">
    <param name="output_frame_id" value="$(arg output_frame_id)"/>
    <param name="input_topics" value="$(arg input_topics)"/>

    <remap from="/points_concat" to="$(arg output_topic)"/>

    <!-- <param name="min_range" value="$(arg min_range)" />
    <param name="max_range" value="$(arg max_range)" /> -->
  </node>

</launch>
```

## new file hpp-points_concat_filter
- points_concat_filter.hpp

```
#ifndef ROS_POINTS_CONCAT_FILTER_H__
#define ROS_POINTS_CONCAT_FILTER_H__

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <velodyne_pointcloud/point_types.h>
#include <yaml-cpp/yaml.h>

class PointsConcatFilter
{
public:
  PointsConcatFilter();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;
  typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT>
      SyncPolicyT;

  ros::NodeHandle node_handle_, private_node_handle_;
  message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];
  message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;
  ros::Subscriber config_subscriber_;
  ros::Publisher cloud_publisher_;
  tf::TransformListener tf_listener_;

  size_t input_topics_size_;
  std::string input_topics_;
  std::string output_frame_id_;

  /* std::string min_range;
  std::string max_range; */

  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                           const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                           const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                           const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
};

#endif
```

## new file cpp-points_concat_filter
- points_concat_filter.cpp

```
/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros-point-cloud-merger/points_concat_filter.hpp"

PointsConcatFilter::PointsConcatFilter() : node_handle_(), private_node_handle_("~"), tf_listener_()
{
  private_node_handle_.param("input_topics", input_topics_, std::string("[/velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]"));
  private_node_handle_.param("output_frame_id", output_frame_id_, std::string("/velodyne_frame"));

  YAML::Node topics = YAML::Load(input_topics_);
  input_topics_size_ = topics.size();
  if (input_topics_size_ < 2 || 8 < input_topics_size_)
  {
    ROS_ERROR("The size of input_topics must be between 2 and 8");
    ros::shutdown();
  }
  for (size_t i = 0; i < 8; ++i)
  {
    if (i < input_topics_size_)
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(node_handle_, topics[i].as<std::string>(), 1);
    }
    else
    {
      cloud_subscribers_[i] =
          new message_filters::Subscriber<PointCloudMsgT>(node_handle_, topics[0].as<std::string>(), 1);
    }
  }
  cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
      SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
      *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);
  cloud_synchronizer_->registerCallback(
      boost::bind(&PointsConcatFilter::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
  cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>("/points_concat", 1);
}

void PointsConcatFilter::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                             const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                             const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                             const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
{
  assert(2 <= input_topics_size_ && input_topics_size_ <= 8);

  PointCloudMsgT::ConstPtr msgs[8] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};
  PointCloudT::Ptr cloud_sources[8];
  PointCloudT::Ptr cloud_concatenated(new PointCloudT);

  // transform points
  try
  {
    for (size_t i = 0; i < input_topics_size_; ++i)
    {
      // Note: If you use kinetic, you can directly receive messages as
      // PointCloutT.

      /* makeShared(): Copy the cloud to the heap and return a smart pointer
        Returns:
        shared pointer to the copy of the cloud */
      cloud_sources[i] = PointCloudT().makeShared();

      /*
        Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
        Parameters:
        msg – the PCLPointCloud2 binary blob
        cloud – the resultant pcl::PointCloud<T> */
      pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);

      /* Block until a transform is possible or it times out
        Parameters:
        target_frame – The frame into which to transform
        source_frame – The frame from which to transform
        time – The time at which to transform
        timeout – How long to block before failing
        polling_sleep_duration – How often to retest if failed
        error_msg – A pointer to a string which will be filled with why the transform failed, if not NULL */
      tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));

      /*
        Transforms (Maintain relationship between multiple coordinate frames overtime) a point cloud in a given target TF frame using a TransformListener.
        Parameters:
        target_frame(output_frame_id_) – the target TF frame the point cloud should be transformed to
        target_time(ros::Time(0)) – the target timestamp
        cloud_in(*cloud_sources[i]) – the input point cloud
        fixed_frame(msgs[i]->header.frame_id) – fixed TF frame
        cloud_out(*cloud_sources[i]) – the output point cloud
        tf_listener(tf_listener_) – a TF listener object */
      pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_sources[i], msgs[i]->header.frame_id, *cloud_sources[i], tf_listener_);
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // merge points
  for (size_t i = 0; i < input_topics_size_; ++i)
  {
    *cloud_concatenated += *cloud_sources[i];
  }

  // publish points
  cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);
  cloud_concatenated->header.frame_id = output_frame_id_;
  cloud_publisher_.publish(cloud_concatenated);
}
```

<br>

# ADDITIONAL NOTES

- after every change in the .cpp or .h file, need to do ```catkin_make```

<br>

# REFERENCES

- [Core Perception - launch file](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/launch/points_concat_filter.launch)
- [Core Perception - cpp](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/nodes/points_concat_filter/points_concat_filter.cpp)

<br>

---

# [GitHub Pages](https://e0425705.github.io/husky_sw/)