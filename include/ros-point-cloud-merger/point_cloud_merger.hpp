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

// ROS Feature
#include <ros/ros.h>

/* #include "ros-point-cloud-merger/point_cloud_merger_node.cpp" */

// STL
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <velodyne_pointcloud/point_types.h>

#include <yaml-cpp/yaml.h>

namespace ros_util
{
    class point_cloud_merger
    {
    public:
        // constructor
        point_cloud_merger();

        // function with nothing in it
        /* void start(); */

        // destructor
        ~point_cloud_merger();

        /* bool checkInside(std::vector<int>, int); */

    private:
        /* The typedef declaration provides a way to declare an identifier as a type alias, to be used to replace a possibly complex type name */
        // individual points(x, y, z, intensity)
        typedef pcl::PointXYZI PointT;
        // collection of individual points = pointcloud
        typedef pcl::PointCloud<PointT> PointCloudT;
        // collection of n dimension points
        typedef sensor_msgs::PointCloud2 PointCloudMsgT;

        /* ApproximateTime: match messages even if they have different time stamps */
        typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                                PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                                PointCloudMsgT, PointCloudMsgT>
            SyncPolicyT;

        /// ROS node private and global handle
        ros::NodeHandle private_nh_;
        ros::NodeHandle global_nh_;

        message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];
        message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;

        /* ros::Subscriber config_subscriber_; */
        ros::Publisher cloud_publisher_;
        
        tf::TransformListener tf_listener_;

        // inputs given by user
        std::string input_topics_;
        std::string output_topic_;

        std::string output_frame_id_;

        double pmin_range_;
        double pmax_range_;
        double nmin_range_;
        double nmax_range_;

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

        /* need to sync - same time */
        void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
    };

} // namespace ros_util

#endif