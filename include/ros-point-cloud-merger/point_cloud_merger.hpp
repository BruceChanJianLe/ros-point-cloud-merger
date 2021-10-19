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

#define MIN_SIZE 2
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>

namespace ros_util
{
    class point_cloud_merger
    {
    public:
        /* Constructor */
        point_cloud_merger();

        /* For unit test purposes */
        point_cloud_merger(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, int input) : tf2_listener_(tfBuffer)
        {
            setXMinValue(x_min);
            setXMaxValue(x_max);
            setYMinValue(y_min);
            setYMaxValue(y_max);
            setZMinValue(z_min);
            setZMaxValue(z_max);
            setInputSize(input);
        }

        void setXMinValue(double x_min_value)
        {
            pmin_range_x_ = x_min_value;
            nmin_range_x_ = -x_min_value;
        }

        void setXMaxValue(double x_max_value)
        {
            pmax_range_x_ = x_max_value;
            nmax_range_x_ = -x_max_value;
        }

        void setYMinValue(double y_min_value)
        {
            pmin_range_y_ = y_min_value;
            nmin_range_y_ = -y_min_value;
        }

        void setYMaxValue(double y_max_value)
        {
            pmax_range_y_ = y_max_value;
            nmax_range_y_ = -y_max_value;
        }

        void setZMinValue(double z_min_value)
        {
            pmin_range_z_ = z_min_value;
        }

        void setZMaxValue(double z_max_value)
        {
            pmax_range_z_ = z_max_value;
        }

        void setInputSize(int input)
        {
            input_size_ = input;
        }

        double getXMinValue()
        {
            return pmin_range_x_;
        }

        double getXMaxValue()
        {
            return pmax_range_x_;
        }

        double getYMinValue()
        {
            return pmin_range_y_;
        }

        double getYMaxValue()
        {
            return pmax_range_y_;
        }

        double getZMinValue()
        {
            return pmin_range_z_;
        }

        double getZMaxValue()
        {
            return pmax_range_z_;
        }

        int getInputSize()
        {
            return input_size_;
        }

        std::string checkInputSize()
        {
            std::string output = "Successful!";

            if (input_size_ < MIN_SIZE || input_size_ > MAX_SIZE)
            {
                output = "Rejected! Out of bound input size.";
            }

            return output;
        }

        /* Destructor */
        ~point_cloud_merger(){};

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

        /* Storage for the retrieved value for whether to enable maximum and minimum range */
        std::string enable_range_flag_;

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

        /* Storage for input size, used in testing */
        int input_size_;

        void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
    };

} // namespace ros_util

#endif