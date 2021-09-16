#ifndef ROS_POINT_CLOUD_MERGER_H__
#define ROS_POINT_CLOUD_MERGER_H__

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

namespace ros_util
{
    class point_cloud_merger
    {
    public:
        // constructor
        point_cloud_merger();

        // function with nothing in it
        void start();

        // destructor
        ~point_cloud_merger();

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

        typedef unsigned long size_t;

        /// ROS node private and global handle
        ros::NodeHandle private_nh_;
        ros::NodeHandle global_nh_;

        // for YAML
        size_t input_topics_size_;

        // inputs given by user
        std::string input_topics_;
        std::string output_topic_;

        std::string output_frame_id_;

        std::string min_range;
        std::string max_range;

        /* need to sync - same time */
    };

} // namespace ros_util

#endif