## Contents
- [HOW TO USE THIS PACKAGE](#how-to-use-this-package)
  - [Launch package](#launch-package)
  - [Launch package with particular configuration METHOD 1](#launch-package-with-particular-configuration-method-1)
  - [Launch package with particular configuration METHOD 2](#launch-package-with-particular-configuration-method-2)
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

>Configured files can be accessed in [here](https://github.com/BruceChanJianLe/ros-point-cloud-merger/blob/branch-merge/config)

## Launch package with particular configuration METHOD 1

```
roslaunch ros-point-cloud-merger point_cloud_merger.launch robot_name:=husky
```
>this will launch package with [husky](https://github.com/BruceChanJianLe/ros-point-cloud-merger/blob/branch-merge/config/husky.yaml) configuration

```
roslaunch ros-point-cloud-merger point_cloud_merger.launch robot_name:=kobuki
```
>this will launch package with [kobuki](https://github.com/BruceChanJianLe/ros-point-cloud-merger/blob/branch-merge/config/kobuki.yaml) configuration

## Launch package with particular configuration METHOD 2

in **point_cloud_merger.launch**
```
<arg name="robot_name" default="husky" />
```

>in this case, husky.yaml is configuration that you want

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

```
<?xml version="1.0"?>

<launch>
    <!-- Replace with the desired config file -->
    <arg name="robot_name" default="husky" />

    <node name="point_cloud_merger_node" output="screen" pkg="ros-point-cloud-merger" type="point_cloud_merger_node">

        <rosparam command="load" file="$(find ros-point-cloud-merger)/config/$(arg robot_name).yaml" />

    </node>
</launch>
```

## New file point_cloud_merger_node.cpp

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

## New file point_cloud_merger.hpp

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

#define MAX_SIZE 8

#include <ros/ros.h>

#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>

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

        /* ROS subscription filter. */
        message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[MAX_SIZE];
        /* Synchronizes incoming channels */
        message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;

        /* Manages an advertisement on a specific topic.  */
        ros::Publisher cloud_publisher_;

        /* Subscribes to message and automatically stores incoming data */
        /* Stores known frames and offers a ROS service, "tf_frames", */
        tf2_ros::Buffer tfBuffer;
        /* Request and receive coordinate frame transform information */
        tf2_ros::TransformListener tf2_listener_;

        /* Storage for the retrieved value for input_topics */
        std::string input_topics_;
        /* Storage for the retrieved value for output_topic */
        std::string output_topic_;

        /* Storage for the retrieved value for output_frame_id */
        std::string output_frame_id_;

        /* Storage for the retrieved value for pmin_range_x */
        double pmin_range_x_;
        /* Storage for the retrieved value for pmax_range_x */
        double pmax_range_x_;
        /* Storage for the retrieved value for nmin_range_x */
        double nmin_range_x_;
        /* Storage for the retrieved value for nmax_range_x */
        double nmax_range_x_;

        /* Storage for the retrieved value for pmin_range_y */
        double pmin_range_y_;
        /* Storage for the retrieved value for pmax_range_y */
        double pmax_range_y_;
        /* Storage for the retrieved value for nmin_range_y */
        double nmin_range_y_;
        /* Storage for the retrieved value for nmax_range_y */
        double nmax_range_y_;

        /* Storage for the retrieved value for pmin_range_z */
        double pmin_range_z_;
        /* Storage for the retrieved value for pmax_range_z */
        double pmax_range_z_;

        void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
    };

} // namespace ros_util

#endif
```

## New file point_cloud_merger.cpp

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

#define MIN_SIZE 2
#define MAX_SIZE 8
#define QUEUE_SIZE 1

static int input_size = 0;

namespace ros_util
{
    point_cloud_merger::point_cloud_merger() : private_nh_("~"), global_nh_(), tf2_listener_(tfBuffer)
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

        /* replace input topics >= input size with 1st input topic */
        for (int i = 0; i < MAX_SIZE; i++)
        {
            if (i >= input_size)
            {
                store_input_topics[i] = store_input_topics[0];
            }
        }
        
        /* steps: subscribe, sync, callback */

        /* message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[MAX_SIZE];
        message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;  */

        /* ROS subscription filter. */
        /* update cloud_subscriber with the PointClouds in the input_topics
            for the one with nothing inside, update with the 1st PointCloud */
        for (int i = 0; i < MAX_SIZE; i++)
            {
            if (i < input_size)
            {
                cloud_subscribers_[i] = new message_filters::Subscriber<PointCloudMsgT>(global_nh_, store_input_topics[i], QUEUE_SIZE);
            }
            else
            {
                cloud_subscribers_[i] = new message_filters::Subscriber<PointCloudMsgT>(global_nh_, store_input_topics[0], QUEUE_SIZE);
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
        cloud_publisher_ = global_nh_.advertise<PointCloudMsgT>(output_topic_, QUEUE_SIZE); 
    }

    void point_cloud_merger::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
    {
        /*  If the condition is true, the program continues normally and 
        if the condition is false, the program is terminated and an error message is displayed.  */
        assert(input_size >= MIN_SIZE && input_size <= MAX_SIZE);

        /* typedef boost::shared_ptr<const PointCloud<PointT> > ConstPtr */
        PointCloudMsgT::ConstPtr msgs[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        PointCloudMsgT::ConstPtr msg[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        /* typedef boost::shared_ptr<PointCloud<PointT> > Ptr */
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
                tfBuffer.lookupTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));

                /* transforms (maintain relationship between multiple coordinate frames overtime) a point cloud in a given target TF frame using a TransformListener. */
                pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_source[i], msgs[i]->header.frame_id, *cloud_source[i], tfBuffer);
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