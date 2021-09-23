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
#define ZERO 0

static int input_size = 0;

namespace ros_util
{
    /* 
     * access modifiers: used to create access modifiers within a class
     * e.g., public:
     *         void set_values(int, int);
     * 
     * inheritance: used to specify a base class while creating a child class 
     * e.g., class Child : Base
     * 
     * initialization lists: delimit the beginning of an initialization list
     * e.g., public:
     *         B(int number = 99) : A(number) { }
     * 
     * https://www.quora.com/What-is-the-meaning-of-in-C++
     */

    // Constructor
    point_cloud_merger::point_cloud_merger() : private_nh_("~"), global_nh_(), tf_listener_()
    {
        /* param function

         * Parameters:
         * param_name – The key to be searched on the parameter server.
         * param_val – Storage for the retrieved value.
         * default_val – Value to use if the server doesn't contain this parameter. 
         */

        /* change the values here if want to change values */
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
        /* if (input_topics_size_ < 2 || input_topics_size_ > 8) */
        if (input_size < MIN_SIZE || input_size > MAX_SIZE)
        {
            ROS_ERROR("Size of input_topics must be between 2 and 8! Exiting now...");
            /* Disconnects everything and unregisters from the master. 
            It is generally not necessary to call this function, as the 
            node will automatically shutdown when all NodeHandles destruct. 
            However, if you want to break out of a spin() loop explicitly, 
            this function allows that. */

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

        /* callback */
        cloud_synchronizer_->registerCallback(
            boost::bind(&point_cloud_merger::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        /* This method returns a Publisher that allows you to publish a message on this topic.
         * Parameters:
         * topic – Topic to advertise on
         * queue_size – Maximum number of outgoing messages to be queued for delivery to subscribers
         */
        cloud_publisher_ = global_nh_.advertise<PointCloudMsgT>(output_topic_, 1);
    }

    /* void point_cloud_merger::start()
    {
    } */

    void point_cloud_merger::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
    {
        /*  If the condition is true, the program continues normally and 
        if the condition is false, the program is terminated and an error message is displayed.  */
        assert(input_size >= MIN_SIZE && input_size <= MAX_SIZE);

        PointCloudMsgT::ConstPtr msgs[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};
        
        // Array
        PointCloudMsgT::ConstPtr msg[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        PointCloudT::Ptr cloud_sources[MAX_SIZE];

        PointCloudT::Ptr cloud_concatenated(new PointCloudT);

        PointCloudT::Ptr cloud_source[MAX_SIZE];

        // transform points
        try
        {
            for (int i = ZERO; i < input_size; i++)
            {
                // Note: If you use kinetic, you can directly receive messages as
                // PointCloutT.

                /* makeShared(): Copy the cloud to the heap and return a smart pointer
                    Returns:
                    shared pointer to the copy of the cloud */
                cloud_sources[i] = PointCloudT().makeShared();
                cloud_source[i] = PointCloudT().makeShared();

                /*
                Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
                Parameters:
                msg – the PCLPointCloud2 binary blob
                cloud – the resultant pcl::PointCloud<T> */
                pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);

                // !!!! DEBUG HERE in cpp one pointer cannot point to 2
                pcl::fromROSMsg(*msg[i], *cloud_source[i]);

                int total = cloud_sources[i]->size();

                std::vector<int> number;
                /* std::vector<int>::iterator it; */
                int numbers_to_remove = 0;

                for (int j = 0; j < total; j++)
                {
                    bool outofbound_flag = false;

                    if ((cloud_sources[i]->points[j].x < nmax_range_x_) || ((cloud_sources[i]->points[j].x > nmin_range_x_) && (cloud_sources[i]->points[j].x < pmin_range_x_)) || (cloud_sources[i]->points[j].x > pmax_range_x_))
                    {
                        outofbound_flag = true;
                        /* cloud_sources[i]->points[j].x = INT_MAX; */
                    }
                    else if ((cloud_sources[i]->points[j].y < nmax_range_y_) || ((cloud_sources[i]->points[j].y > nmin_range_y_) && (cloud_sources[i]->points[j].y < pmin_range_y_)) || (cloud_sources[i]->points[j].y > pmax_range_y_))
                    {
                        outofbound_flag = true;
                        /* cloud_sources[i]->points[j].y = INT_MAX; */
                    }
                    /* else if ((cloud_sources[i]->points[j].z < pmin_range_z_) || (cloud_sources[i]->points[j].z > pmax_range_z_))
                    {
                        outofbound_flag = true;
                    } */

                    if (outofbound_flag == true)
                    {
                        // add points into vector
                        number.insert(number.end(), j);
                        numbers_to_remove++; // or can number.size()
                    }
                }

                // Remove the points by setting to inf
                /* for (int j = 0; j < numbers_to_remove; j++)
                {
                    cloud_sources[i]->points[number[j]].x = INT_MAX;
                    cloud_sources[i]->points[number[j]].y = INT_MAX;
                    cloud_sources[i]->points[number[j]].z = INT_MAX;
                } */

                // remove out of bound points
                cloud_source[i]->clear();

                int j = 0;
                for (int k = 0; k < cloud_sources[i]->size(); k++)
                {
                    /* if (checkInside(number, j) == false)
                    {
                        cloud_source[i]->points[j] = cloud_sources[i]->points[k];
                    } */
                    bool is_inside = false;
                    for (int l = 0; l < number.size(); l++)
                    {
                        if (k == number[l])
                        {
                            is_inside = true;
                        }
                    }
                    // if index is not in vector
                    if (is_inside == false)
                    {
                        cloud_source[i]->points[j] = cloud_sources[i]->points[k];
                    }
                    j++;
                }

                cloud_sources[i].swap(cloud_source[i]);

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

    // Destructor
    point_cloud_merger::~point_cloud_merger()
    {
    }

    /* bool point_cloud_merger::checkInside(std::vector<int> number, int current)
    {
        for (int i = 0; i < number.size(); i++)
        {
            if (current = number[i])
            {
                return true;
            }
        }
        return false;
    } */

} // namespace ros_util