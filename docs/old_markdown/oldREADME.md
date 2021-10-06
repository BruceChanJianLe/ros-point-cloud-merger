## Contents
- [HOW TO USE THIS PACKAGE](#how-to-use-this-package)
  - [Launch package](#launch-package)
- [HOW TO CHANGE VARIABLES](#how-to-change-variables)
  - [Change input topics in **point_cloud_merger.launch**](#change-input-topics-in-point_cloud_mergerlaunch)
  - [Change output topic in **point_cloud_merger.launch**](#change-output-topic-in-point_cloud_mergerlaunch)
  - [Change output frame id in **point_cloud_merger.launch**](#change-output-frame-id-in-point_cloud_mergerlaunch)
  - [Change minimum range x axis in **point_cloud_merger.launch**](#change-minimum-range-x-axis-in-point_cloud_mergerlaunch)
  - [Change minimum range y axis in **point_cloud_merger.launch**](#change-minimum-range-y-axis-in-point_cloud_mergerlaunch)
  - [Change maximum range x axis in **point_cloud_merger.launch**](#change-maximum-range-x-axis-in-point_cloud_mergerlaunch)
  - [Change maximum range y axis](#change-maximum-range-y-axis)
  - [Change minimum height z axis](#change-minimum-height-z-axis)
  - [Change maximum height z axis](#change-maximum-height-z-axis)
- [WHAT EXISTING FILES TO CHANGE](#what-existing-files-to-change)
  - [How package.xml should look like](#how-packagexml-should-look-like)
  - [How CMakeLists.txt should look like](#how-cmakeliststxt-should-look-like)
  - [Add in robot urdf](#add-in-robot-urdf)
- [NEW FILES TO ADD IN](#new-files-to-add-in)
  - [New file launch-point-cloud-merger](#new-file-launch-point-cloud-merger)
  - [New file cpp-point-cloud-merger-node](#new-file-cpp-point-cloud-merger-node)
  - [New file hpp-point_cloud_merger](#new-file-hpp-point_cloud_merger)
  - [New file cpp-point_cloud_merger](#new-file-cpp-point_cloud_merger)
- [EXAMPLE IMPLEMENTATION](#example-implementation)
- [REFERENCES](#references)

<br>

# HOW TO USE THIS PACKAGE

## Launch package

```
roslaunch ros-point-cloud-merger point_cloud_merger.launch
```

<br>

# HOW TO CHANGE VARIABLES

## Change input topics in **point_cloud_merger.launch**

in this case, **/alpha** and **/beta** are the pointclouds that you want to merge
```
<arg name="input_topics" default="[/alpha, /beta]" />
```

## Change output topic in **point_cloud_merger.launch**

in this case, **/points_concat** is output topic name for the merged pointcloud
```
<arg name="output_topic" default="/points_concat" />
```

## Change output frame id in **point_cloud_merger.launch**

in this case, **velodyne_frame** is output frame id for the merged pointcloud
```
<arg name="output_frame_id" default="velodyne_frame" />
```

## Change minimum range x axis in **point_cloud_merger.launch**

in this case, **0.5** and **-0.5** are desired minimum range in the positive x axis and negative x axis respectively
```
<arg name="pmin_range_x" default="0.5" />
<arg name="nmin_range_x" default="-0.5" />
```

## Change minimum range y axis in **point_cloud_merger.launch**

in this case, **0.5** and **-0.5** are desired minimum range in the positive y axis and negative y axis respectively
```
<arg name="pmin_range_y" default="0.5" />
<arg name="nmin_range_y" default="-0.5" />
```

## Change maximum range x axis in **point_cloud_merger.launch**

in this case, **5.0** and **-5.0** are desired maximum range in the positive x axis and negative x axis respectively
```
<arg name="pmax_range_x" default="5.0" />
<arg name="nmax_range_x" default="-5.0" />
```

## Change maximum range y axis

**in point_cloud_merger.launch**

in this case, **5.0** and **-5.0** are desired maximum range in the positive y axis and negative y axis respectively
```
<arg name="pmax_range_y" default="5.0" />
<arg name="nmax_range_y" default="-5.0" />
```

## Change minimum height z axis

**in point_cloud_merger.launch**

in this case, **-1.0** is desired minimum height in the z axis 
```
<arg name="pmin_range_z" default="-1.0" />
```

## Change maximum height z axis

**in point_cloud_merger.launch**

in this case, **100.0** is desired maximum height in the z axis
```
<arg name="pmax_range_z" default="100.0" />
```

<br>

# WHAT EXISTING FILES TO CHANGE

## How package.xml should look like
- **package.xml**

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
  <build_depend>rospy</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>tf</build_depend>

  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>message_filters</build_export_depend>
  <build_export_depend>pcl_conversions</build_export_depend>
  <build_export_depend>pcl_ros</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>sensor_msgs</build_export_depend>
  <build_export_depend>tf</build_export_depend>

  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_filters</exec_depend>
  <exec_depend>pcl_conversions</exec_depend>
  <exec_depend>pcl_ros</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>tf</exec_depend>

  <export>
  </export>
</package>
```

## How CMakeLists.txt should look like
- **CMakeLists.txt**

```
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

install(
  TARGETS
    point_cloud_merger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## Add in robot urdf

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

<br>

# NEW FILES TO ADD IN

## New file launch-point-cloud-merger
- **point_cloud_merger.launch**

```
<?xml version="1.0"?>

<launch>
    <!-- replace with the desired input topics -->
    <arg name="input_topics" default="[/velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]" />
    <!-- replace with the desired output topic -->
    <arg name="output_topic" default="/points_concat" />
    <!-- replace with the desired output frame 
         to change in the link to robot.urdf as well -->
    <arg name="output_frame_id" default="velodyne_frame" />

    <!-- replace with the desired positive x min range from base_link -->
    <arg name="pmin_range_x" default="0.5" />
    <!-- replace with the desired negative x min range from base_link -->
    <arg name="nmin_range_x" default="-0.5" />
    <!-- replace with the desired positive x max range from base_link -->
    <arg name="pmax_range_x" default="5.0" />
    <!-- replace with the desired negative x max range from base_link -->
    <arg name="nmax_range_x" default="-5.0" />

    <!-- replace with the desired positive y min range from base_link -->
    <arg name="pmin_range_y" default="0.5" />
    <!-- replace with the desired negative y min range from base_link -->
    <arg name="nmin_range_y" default="-0.5" />
    <!-- replace with the desired positive y max range from base_link -->
    <arg name="pmax_range_y" default="5.0" />
    <!-- replace with the desired negative y max range from base_link -->
    <arg name="nmax_range_y" default="-5.0" />

    <!-- replace with the desired z min height -->
    <arg name="pmin_range_z" default="-1.0" />
    <!-- replace with the desired z max height -->
    <arg name="pmax_range_z" default="100.0" />


    <node name="point_cloud_merger_node" output="screen" pkg="ros-point-cloud-merger" type="point_cloud_merger_node">
        <param name="output_frame_id" value="$(arg output_frame_id)"/>

        <param name="input_topics" value="$(arg input_topics)"/>
        <param name="output_topic" value="$(arg output_topic)"/>

        <param name="pmin_range_x" type="double" value="$(arg pmin_range_x)" />
        <param name="pmax_range_x" type="double" value="$(arg pmax_range_x)" />
        <param name="nmin_range_x" type="double" value="$(arg nmin_range_x)" />
        <param name="nmax_range_x" type="double" value="$(arg nmax_range_x)" />

        <param name="pmin_range_y" type="double" value="$(arg pmin_range_y)" />
        <param name="pmax_range_y" type="double" value="$(arg pmax_range_y)" />
        <param name="nmin_range_y" type="double" value="$(arg nmin_range_y)" />
        <param name="nmax_range_y" type="double" value="$(arg nmax_range_y)" />

        <param name="pmin_range_z" type="double" value="$(arg pmin_range_z)" />
        <param name="pmax_range_z" type="double" value="$(arg pmax_range_z)" />
    </node>

</launch>
```

## New file cpp-point-cloud-merger-node
- **point_cloud_merger_node.cpp**

```
#include "ros-point-cloud-merger/point_cloud_merger.hpp"

#include <ros/ros.h>

const std::string ROSNodeName = "point_cloud_merger_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROSNodeName);

    ros_util::point_cloud_merger node;

    ros::spin();

    return 0;
}
```

## New file hpp-point_cloud_merger
- **point_cloud_merger.hpp**

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

#ifndef ROS_POINT_CLOUD_MERGER_H__
#define ROS_POINT_CLOUD_MERGER_H__

#include <ros/ros.h>

#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace ros_util
{
    class point_cloud_merger
    {
    public:
        point_cloud_merger();
        ~point_cloud_merger();

    private:
        typedef pcl::PointXYZI PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef sensor_msgs::PointCloud2 PointCloudMsgT;

        typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                                PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                                PointCloudMsgT, PointCloudMsgT>
            SyncPolicyT;

        ros::NodeHandle private_nh_;
        ros::NodeHandle global_nh_;

        message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];
        message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;

        ros::Publisher cloud_publisher_;
        
        tf::TransformListener tf_listener_;

        std::string input_topics_;
        std::string output_topic_;

        std::string output_frame_id_;

        double pmin_range_x_;
        double pmax_range_x_;
        double nmin_range_x_;
        double nmax_range_x_;

        double pmin_range_y_;
        double pmax_range_y_;
        double nmin_range_y_;
        double nmax_range_y_;

        double pmin_range_z_;
        double pmax_range_z_;

        void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
    };

} // namespace ros_util

#endif
```

## New file cpp-point_cloud_merger
- **point_cloud_merger.cpp**

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

#include "ros-point-cloud-merger/point_cloud_merger.hpp"
#include <bits/stdc++.h>

#define MIN_SIZE 2
#define MAX_SIZE 8

static int input_size = 0;

namespace ros_util
{
    point_cloud_merger::point_cloud_merger() : private_nh_("~"), global_nh_(), tf_listener_()
    {
        private_nh_.param("input_topics", input_topics_, std::string("[/velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]"));
        private_nh_.param("output_frame_id", output_frame_id_, std::string("/velodyne_frame"));
        private_nh_.param("output_topic", output_topic_, std::string("/points_concat"));

        private_nh_.param("pmin_range_x", pmin_range_x_, double(0.9));
        private_nh_.param("pmax_range_x", pmax_range_x_, double(2.0));
        private_nh_.param("nmin_range_x", nmin_range_x_, double(-0.9));
        private_nh_.param("nmax_range_x", nmax_range_x_, double(-2.0));

        private_nh_.param("pmin_range_y", pmin_range_y_, double(0.9));
        private_nh_.param("pmax_range_y", pmax_range_y_, double(2.0));
        private_nh_.param("nmin_range_y", nmin_range_y_, double(-0.9));
        private_nh_.param("nmax_range_y", nmax_range_y_, double(-2.0));

        private_nh_.param("pmin_range_z", pmin_range_z_, double(-1.0));
        private_nh_.param("pmax_range_z", pmax_range_z_, double(100.0));

        input_topics_ = input_topics_.substr(1);
        std::stringstream ss(input_topics_);
        std::string source;

        /* 
         *   in this case do not need to store input topics into a array  
         *   but can be useful in future projects
         */
        std::string store_input_topics[MAX_SIZE];

        while (ss >> source)
        {
            source = source.substr(0, source.length() - 1);
            store_input_topics[input_size] = source;
            input_size++;
        }

        // check number of input topics accepted
        if (input_size < MIN_SIZE)
        {
            ROS_ERROR("Minimum size accepted is 2 but size of input topics is less than 2. Exiting now...");

            /* Disconnects everything and unregisters from the master. 
            It is generally not necessary to call this function, as the 
            node will automatically shutdown when all NodeHandles destruct. 
            However, if you want to break out of a spin() loop explicitly, 
            this function allows that. */
            ros::shutdown();

            return;
        }
        else if (input_size > MAX_SIZE)
        {
            ROS_ERROR("Maximum size accepted is 8 but size of input topics is more than 8. Exiting now...");

            /* Disconnects everything and unregisters from the master. 
            It is generally not necessary to call this function, as the 
            node will automatically shutdown when all NodeHandles destruct. 
            However, if you want to break out of a spin() loop explicitly, 
            this function allows that. */
            ros::shutdown();

            return;
        }

        /* steps: subscribe, sync, callback */
        for (int i = 0; i < MAX_SIZE; i++)
        {
            /* update cloud_subscriber with the PointClouds in the input_topics
            for the one with nothing inside, update with the 1st PointCloud */
            if (i < input_size)
            {
                cloud_subscribers_[i] =
                    new message_filters::Subscriber<PointCloudMsgT>(global_nh_, store_input_topics[i], 1);
            }
            else
            {
                cloud_subscribers_[i] =
                    new message_filters::Subscriber<PointCloudMsgT>(global_nh_, store_input_topics[0], 1);
            }
        }

        /* Sychronise the cloud_subscribers */
        cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
            SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
            *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);

        /* callback */
        cloud_synchronizer_->registerCallback(
            boost::bind(&point_cloud_merger::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        /* returns a Publisher that allows you to publish a message on this topic. */
        cloud_publisher_ = global_nh_.advertise<PointCloudMsgT>(output_topic_, 1);
    }

    void point_cloud_merger::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
    {
        /*  If the condition is true, the program continues normally and 
        if the condition is false, the program is terminated and an error message is displayed.  */
        assert(input_size >= MIN_SIZE && input_size <= MAX_SIZE);

        /* typedef ConstPtr boost::shared_ptr<const PointCloud<pcl::PointXYZ>> */
        PointCloudMsgT::ConstPtr msgs[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        PointCloudMsgT::ConstPtr msg[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        boost::shared_ptr<PointCloudT> cloud_sources[MAX_SIZE];

        boost::shared_ptr<PointCloudT> cloud_source[MAX_SIZE];

        boost::shared_ptr<PointCloudT> cloud_concatenated(new PointCloudT);

        // transform points
        try
        {
            for (int i = 0; i < input_size; i++)
            {
                // Note: If you use kinetic, you can directly receive messages as
                // PointCloutT.

                /* copy the cloud to the heap and return a smart pointer */
                cloud_sources[i] = PointCloudT().makeShared();
                cloud_source[i] = PointCloudT().makeShared();

                /* convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map. */
                pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);
                pcl::fromROSMsg(*msg[i], *cloud_source[i]);

                int current_index = 0;

                for (int j = 0; j < cloud_sources[i]->size(); j++)
                {
                    bool outofbound_flag = false;

                    if ((cloud_sources[i]->points[j].x > pmax_range_x_) || (cloud_sources[i]->points[j].y > pmax_range_y_))
                    {
                        outofbound_flag = true;
                    }
                    else if ((cloud_sources[i]->points[j].x < nmax_range_x_) || (cloud_sources[i]->points[j].y < nmax_range_y_))
                    {
                        outofbound_flag = true;
                    }
                    else if (((cloud_sources[i]->points[j].x > nmin_range_x_) && (cloud_sources[i]->points[j].x < pmin_range_x_)) && ((cloud_sources[i]->points[j].y > nmin_range_y_) && (cloud_sources[i]->points[j].y < pmin_range_y_)))
                    {
                        outofbound_flag = true;
                    }
                    else if ((cloud_sources[i]->points[j].z < pmin_range_z_) || (cloud_sources[i]->points[j].z > pmax_range_z_))
                    {
                        outofbound_flag = true;
                    }
                    else if (outofbound_flag == false)
                    {
                        cloud_source[i]->points[current_index].x = cloud_sources[i]->points[j].x;
                        cloud_source[i]->points[current_index].y = cloud_sources[i]->points[j].y;
                        cloud_source[i]->points[current_index].z = cloud_sources[i]->points[j].z;
                        current_index++;
                    }
                }

                /* set the points that are out of bound as INT_MAX */
                if (cloud_source[i]->size() > current_index)
                {
                    for (int p = current_index; p < cloud_source[i]->size(); p++)
                    {
                        cloud_source[i]->points[p].x = INT_MAX;
                        cloud_source[i]->points[p].y = INT_MAX;
                        cloud_source[i]->points[p].z = INT_MAX;
                    }
                }

                /* block until a transform is possible or it times out */
                tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));

                /* transforms (maintain relationship between multiple coordinate frames overtime) a point cloud in a given target TF frame using a TransformListener. */
                pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_source[i], msgs[i]->header.frame_id, *cloud_source[i], tf_listener_);
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());

            /* return 0; means program exited successful and atleast in (unix) 
            while return just terminates the program regardles of it's state. */
            return;
        }

        // merge points
        for (int i = 0; i < input_size; i++)
        {
            *cloud_concatenated += *cloud_source[i];
        }

        // publish points
        cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);

        cloud_concatenated->header.frame_id = output_frame_id_;

        /* publish a message on the topic associated with this Publisher. */
        cloud_publisher_.publish(cloud_concatenated);
    }

    point_cloud_merger::~point_cloud_merger()
    {
    }

} // namespace ros_util
```

<br>

# EXAMPLE IMPLEMENTATION

[example](https://github.com/e0425705/husky_sw/tree/branch-package)

<br>

# REFERENCES

- [Core Perception - launch file](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/launch/points_concat_filter.launch)
- [Core Perception - cpp](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/nodes/points_concat_filter/points_concat_filter.cpp)