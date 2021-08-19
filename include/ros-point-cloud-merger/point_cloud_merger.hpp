#ifndef ROS_POINT_CLOUD_MERGER_H__
#define ROS_POINT_CLOUD_MERGER_H__

// ROS Feature
#include <ros/ros.h>

// STL
#include <string>

namespace ros_util
{
    class point_cloud_merger
    {
    private:
        /// ROS node private and globla handle
        ros::NodeHandle private_nh_;
        ros::NodeHandle global_nh_;

    public:
        point_cloud_merger();
        ~point_cloud_merger();
    };

} // namespace ros_util

#endif