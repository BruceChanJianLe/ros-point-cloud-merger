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

#include <bits/stdc++.h>

// ROS Feature
#include <ros/ros.h>

#include <stdio.h>

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

#define MIN_SIZE 2
#define MAX_SIZE 8
#define ZERO 0

static int input_size = 0;

namespace ros_util
{
    class point_cloud_merger
    {
    public:
        // constructor
        point_cloud_merger();

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

        ros::Subscriber config_subscriber_;
        ros::Publisher cloud_publisher_;

        tf::TransformListener tf_listener_;

        // inputs given by user
        std::string input_topics_;
        std::string output_topic_;

        std::string output_frame_id_;

        std::string pmin_range_x_;
        std::string pmax_range_x_;
        std::string nmin_range_x_;
        std::string nmax_range_x_;

        std::string pmin_range_y_;
        std::string pmax_range_y_;
        std::string nmin_range_y_;
        std::string nmax_range_y_;

        std::string pmin_range_z_;
        std::string pmax_range_z_;

        /* std::string pmin_range_;
        std::string pmax_range_;
        std::string nmin_range_;
        std::string nmax_range_; */

        /* need to sync - same time */
        void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
    };

    // Constructor
    point_cloud_merger::point_cloud_merger() : private_nh_("~"), global_nh_(), tf_listener_()
    {
        private_nh_.param("input_topics", input_topics_, std::string("[/velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]"));
        private_nh_.param("output_frame_id", output_frame_id_, std::string("/velodyne_frame"));
        private_nh_.param("output_topic", output_topic_, std::string("/points_concat"));

        private_nh_.param("pmin_range_x", pmin_range_x_, std::string("0.9"));
        private_nh_.param("pmax_range_x", pmax_range_x_, std::string("10.0"));
        private_nh_.param("nmin_range_x", nmin_range_x_, std::string("-0.9"));
        private_nh_.param("nmax_range_x", nmax_range_x_, std::string("-10.0"));

        private_nh_.param("pmin_range_y", pmin_range_y_, std::string("0.9"));
        private_nh_.param("pmax_range_y", pmax_range_y_, std::string("10.0"));
        private_nh_.param("nmin_range_y", nmin_range_y_, std::string("-0.9"));
        private_nh_.param("nmax_range_y", nmax_range_y_, std::string("-10.0"));

        private_nh_.param("pmin_range_z", pmin_range_z_, std::string("0."));
        private_nh_.param("pmax_range_z", pmax_range_z_, std::string("100.0"));

        /* namespace YAML, class Node in library yaml-cpp */
        /* YAML::Node YAML::Load(const std::string &input) */
        YAML::Node topics = YAML::Load(input_topics_);

        /* 
         * Array of input topics: store_input_topics[]
         * Number of topics: input_size
         */
        std::string store_input_topics[8];
        bool is_last_topic = false;

        /* not good practice, no error catching */
        /* have to strictly adhere to format for input topic */
        while (input_topics_.compare("") > 0)
        {
            int start_of_topic, end_of_topic;
            start_of_topic = input_topics_.find_first_of('/');

            if (start_of_topic > 0)
            {
                end_of_topic = input_topics_.find_first_of(',');

                /* reached last topic */
                if (end_of_topic < 0)
                {
                    end_of_topic = input_topics_.find_last_of(']');
                    is_last_topic = true;
                }
            }

            store_input_topics[input_size] = input_topics_.substr(start_of_topic, --end_of_topic);
            input_topics_ = input_topics_.substr(end_of_topic + 2);

            if (is_last_topic == true)
            {
                input_topics_ = "";
            }
            input_size++;
        }

        // check range of input topics accepted
        if (input_size < MIN_SIZE || input_size > MAX_SIZE)
        {
            ROS_ERROR("Size of input_topics must be between 2 and 8! Exiting now...");

            ros::shutdown();
        }

        /* steps: subscribe, sync, callback */
        for (int i = ZERO; i < MAX_SIZE; i++)
        {
            /* update cloud_subscriber with the PointClouds in the input_topics
                for the one with nothing inside, update with the 1st PointCloud */
            if (i < input_size)
            {
                /* Constructor See the ros::NodeHandle::subscribe() variants for more information on the parameters */
                cloud_subscribers_[i] =
                    new message_filters::Subscriber<PointCloudMsgT>(global_nh_, topics[i].as<std::string>(), 1);
            }
            else
            {
                cloud_subscribers_[i] =
                    new message_filters::Subscriber<PointCloudMsgT>(global_nh_, topics[0].as<std::string>(), 1);
            }
        }

        /* Sychronise the cloud_subscribers */
        cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
            SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
            *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);

        cloud_synchronizer_->registerCallback(
            boost::bind(&point_cloud_merger::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        cloud_publisher_ = global_nh_.advertise<PointCloudMsgT>(output_topic_, 1);
    }

    void point_cloud_merger::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
    {
        assert(input_size >= MIN_SIZE && input_size <= MAX_SIZE);

        PointCloudMsgT::ConstPtr msgs[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        PointCloudT::Ptr cloud_sources[MAX_SIZE];

        PointCloudT::Ptr cloud_concatenated(new PointCloudT);

        // transform points
        try
        {
            for (int i = ZERO; i < input_size; i++)
            {
                cloud_sources[i] = PointCloudT().makeShared();

                pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);

                int total = cloud_sources[i]->size();

                std::vector<int> number;
                /* std::vector<int>::iterator it; */
                int numbers_to_remove = 0;

                for (int j = 0; j < total; j++)
                {
                    bool outofbound_flag = false;

                    if ((cloud_sources[i]->points[j].x < stoi(nmax_range_x_)) || ((cloud_sources[i]->points[j].x > stoi(nmin_range_x_)) && (cloud_sources[i]->points[j].x < stoi(pmin_range_x_))) || ((cloud_sources[i]->points[j].x > stoi(nmin_range_x_)) && (cloud_sources[i]->points[j].x > stoi(pmax_range_x_))))
                    {
                        /* cloud_sources[i]->points[j].x = INT_MAX; */
                        outofbound_flag = true;
                    }
                    else if ((cloud_sources[i]->points[j].y < stoi(nmax_range_y_)) || ((cloud_sources[i]->points[j].y > stoi(nmin_range_y_)) && (cloud_sources[i]->points[j].y < stoi(pmin_range_y_))) || ((cloud_sources[i]->points[j].y > stoi(nmin_range_y_)) && (cloud_sources[i]->points[j].y > stoi(pmax_range_y_))))
                    {
                        /* cloud_sources[i]->points[j].y = INT_MAX; */
                        outofbound_flag = true;
                    }
                    /* else if ((cloud_sources[i]->points[j].z < stoi(pmin_range_z_)) || (cloud_sources[i]->points[j].z > stoi(pmax_range_z_)))
                    { */
                        /* cloud_sources[i]->points[j].z = INT_MAX; */
                        /* outofbound_flag = true;
                    } */

                    if (outofbound_flag == true)
                    {
                        /* it = number.begin();
                        number.insert(it, j); */

                        number.insert(number.begin(), j);
                        numbers_to_remove++;
                    }
                }

                // Remove the points
                for (int j = 0; j < numbers_to_remove; j++)
                {
                    cloud_sources[i]->points[number[j]].x = INT_MAX;
                    cloud_sources[i]->points[number[j]].y = INT_MAX;
                    cloud_sources[i]->points[number[j]].z = INT_MAX;
                }

                tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));

                pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_sources[i], msgs[i]->header.frame_id, *cloud_sources[i], tf_listener_);
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
        /* for (size_t i = 0; i < input_topics_size_; i++) */
        for (int i = ZERO; i < input_size; i++)
        {
            *cloud_concatenated += *cloud_sources[i];
        }

        // publish points
        cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);

        cloud_concatenated->header.frame_id = output_frame_id_;

        /* Publish a message on the topic associated with this Publisher. */
        cloud_publisher_.publish(cloud_concatenated);
    }

    /* const std::string ROSNodeName = "point_cloud_merger"; */

    int main(int argc, char **argv)
    {
        ros::init(argc, argv, "point_cloud_merger");
        ros_util::point_cloud_merger node;
        ros::spin();

        return 0;
    }

} // namespace ros_util