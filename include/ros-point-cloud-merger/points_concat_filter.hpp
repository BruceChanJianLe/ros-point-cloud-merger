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

#ifndef ROS_POINTS_CONCAT_FILTER_H__
#define ROS_POINTS_CONCAT_FILTER_H__

// ROS Feature
#include <ros/ros.h>

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

class PointsConcatFilter
{
public:
      PointsConcatFilter();
      ~PointsConcatFilter();

private:
      // individual points
      typedef pcl::PointXYZI PointT;
      // individual points making a pointcloud
      typedef pcl::PointCloud<PointT> PointCloudT;
      /* This message holds a collection of N-dimensional points, which may
       * contain additional information such as normals, intensity, etc. The
       * point data is stored as a binary blob, its layout described by the
       * contents of the "fields" array. */
      typedef sensor_msgs::PointCloud2 PointCloudMsgT;

      /* ApproximateTime: match messages even if they have different time stamps */
      typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                              PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                              PointCloudMsgT, PointCloudMsgT>
          SyncPolicyT;

      ros::NodeHandle node_handle_, private_node_handle_;

      /* message_filters: A set of message filters which take in messages 
       * and may output those messages at a later time, 
       * based on the conditions that filter needs met. */

      /* Manages an advertisement on a specific topic. A Publisher should always be created through a call to 
        NodeHandle::advertise(), or copied from one that was. Once all copies of a specific Publisher go out of 
        scope, any subscriber status callbacks associated with that handle will stop being called. Once all 
        Publishers for a given topic go out of scope the topic will be unadvertised. */
      // create subscribers
      message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];

      /* typedef */
      /* sync the subscribers which is typedef earlier */
      message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;

      /* Manages an subscription callback on a specific topic. A Subscriber should always be created 
            through a call to NodeHandle::subscribe(), or copied from one that was. Once all copies of 
            a specific Subscriber go out of scope, the subscription callback associated with that 
            handle will stop being called. Once all Subscriber for a given topic go out of scope the 
            topic will be unsubscribed. */
      // create a ros::Subscriber which is used to subscribe to a topic
      ros::Subscriber config_subscriber_;

      /* Manages an advertisement on a specific topic. A Publisher should always be created through 
        a call to NodeHandle::advertise(), or copied from one that was. Once all copies of a specific 
        Publisher go out of scope, any subscriber status callbacks associated with that handle will 
        stop being called. Once all Publishers for a given topic go out of scope the topic will 
        be unadvertised. */
      // create a ros::Publisher which is used to publish on a topic
      ros::Publisher cloud_publisher_;

      /* subscribes to message and automatically stores incoming data */
      tf::TransformListener tf_listener_;

      // typedef unsigned long size_t
      size_t input_topics_size_;

      std::string input_topics_;
      std::string output_topic_;

      std::string output_frame_id_;

      /* std::string pmin_range_x_;
      std::string pmax_range_x_;
      std::string nmin_range_x_;
      std::string nmax_range_x_;

      std::string pmin_range_y_;
      std::string pmax_range_y_;
      std::string nmin_range_y_;
      std::string nmax_range_y_;

      std::string pmin_range_z_;
      std::string pmax_range_z_; */

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

      /* A callback is a function that is to be executed after another function has finished executing */
      /* void pointcloud_callback(PointCloudMsgT::Ptr &msg1, PointCloudMsgT::Ptr &msg2,
                           PointCloudMsgT::Ptr &msg3, PointCloudMsgT::Ptr &msg4,
                           PointCloudMsgT::Ptr &msg5, PointCloudMsgT::Ptr &msg6,
                           PointCloudMsgT::Ptr &msg7, PointCloudMsgT::Ptr &msg8); */
      void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                               const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                               const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                               const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
};

#endif