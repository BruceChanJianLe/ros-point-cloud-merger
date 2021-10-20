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

namespace ros_util
{
    /* point_cloud_merger::point_cloud_merger(std::string inputs, std::string output_frame_id, std::string output, double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, int input_size) 
    : private_nh_("~"), global_nh_(), tf2_listener_(tfBuffer),
    input_topics_(inputs), output_frame_id_(output_frame_id), output_topic_(output),
    pmin_range_x_(x_min), nmin_range_x_(-x_min), pmin_range_y_(y_min), nmin_range_y_(-y_min),
    pmax_range_x_(x_max), nmax_range_x_(-x_max), pmax_range_y_(y_max), nmax_range_y_(-y_max), 
    pmin_range_z_(z_min), pmax_range_z_(z_max), set_input_size_(input_size) */
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

        private_nh_.param("enable_range_flag", enable_range_flag_, false);
        private_nh_.param("input_size", input_size_, int(0));

        /* If enable_range_flag is not enabled */
        /* if (enable_range_flag_.compare("true") != 0) */
        if (enable_range_flag_ == false)
        {
            pmin_range_x_ = 0.0;
            pmax_range_x_ = DBL_MAX;
            nmin_range_x_ = 0.0;
            nmax_range_x_ = -DBL_MAX;

            pmin_range_y_ = 0.0;
            pmax_range_y_ = DBL_MAX;
            nmin_range_y_ = 0.0;
            nmax_range_y_ = -DBL_MAX;

            pmin_range_z_ = -DBL_MAX;
            pmax_range_z_ = DBL_MAX;
        }

        input_topics_ = input_topics_.substr(1);
        std::stringstream ss(input_topics_);
        std::string source;

        /* Array that stores input topics */
        std::string store_input_topics[MAX_SIZE];

        /* Stores input into store_input_topics[] array */
        while (ss >> source)
        {
            source = source.substr(0, source.length() - 1);
            store_input_topics[input_size_] = source;
            input_size_++;
        }

        // Check number of input topics accepted
        if (input_size_ < MIN_SIZE)
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
        else if (input_size_ > MAX_SIZE)
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

        /* Replace input topics >= input size with 1st input topic */
        for (int i = 0; i < MAX_SIZE; i++)
        {
            if (i >= input_size_)
            {
                store_input_topics[i] = store_input_topics[0];
            }
        }

        /* Steps: subscribe, sync, callback */

        /* ROS subscription filter. */
        /* Update cloud_subscriber with the PointClouds in the input_topics
            for the one with nothing inside, update with the 1st PointCloud */
        for (int i = 0; i < MAX_SIZE; i++)
        {
            if (i < input_size_)
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

        /* Callback */
        cloud_synchronizer_->registerCallback(
            boost::bind(&point_cloud_merger::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        /* Returns a Publisher that allows you to publish a message on this topic. */
        cloud_publisher_ = global_nh_.advertise<PointCloudMsgT>(output_topic_, QUEUE_SIZE);
    }

    void point_cloud_merger::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
    {
        /*  If the condition is true, the program continues normally and 
        if the condition is false, the program is terminated and an error message is displayed.  */
        assert(input_size_ >= MIN_SIZE && input_size_ <= MAX_SIZE);

        /* typedef boost::shared_ptr<const PointCloud<PointT> > ConstPtr */
        PointCloudMsgT::ConstPtr msgs[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        PointCloudMsgT::ConstPtr msg[MAX_SIZE] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};

        /* typedef boost::shared_ptr<PointCloud<PointT> > Ptr */
        boost::shared_ptr<PointCloudT> cloud_sources[MAX_SIZE];

        boost::shared_ptr<PointCloudT> cloud_source[MAX_SIZE];

        boost::shared_ptr<PointCloudT> cloud_concatenated(new PointCloudT);

        // Transform points
        try
        {
            for (int i = 0; i < input_size_; i++)
            {
                // Note: If you use kinetic, you can directly receive messages as PointCloutT.

                /* Copy the cloud to the heap and return a smart pointer */
                cloud_sources[i] = PointCloudT().makeShared();
                cloud_source[i] = PointCloudT().makeShared();

                /* Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map. */
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

                /* Resize pointcloud if current index is > 0 */
                if (current_index > 0)
                {
                    cloud_source[i]->resize(--current_index);
                }
                else
                {
                    ROS_ERROR("PointCloud after filtering is 0. Exiting now...");

                    /* Disconnects everything and unregisters from the master. 
                       It is generally not necessary to call this function, as the 
                       node will automatically shutdown when all NodeHandles destruct. 
                       However, if you want to break out of a spin() loop explicitly, 
                       this function allows that. */

                    return;
                }

                /* Block until a transform is possible or it times out */
                tfBuffer.lookupTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));

                /* Transforms (maintain relationship between multiple coordinate frames overtime) a point cloud in a given target TF frame using a TransformListener. */
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

        // Merge points
        for (int i = 0; i < input_size_; i++)
        {
            *cloud_concatenated += *cloud_source[i];
        }

        // Publish points
        cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);

        cloud_concatenated->header.frame_id = output_frame_id_;

        /* Publish a message on the topic associated with this Publisher. */
        cloud_publisher_.publish(cloud_concatenated);
    }

} // namespace ros_util