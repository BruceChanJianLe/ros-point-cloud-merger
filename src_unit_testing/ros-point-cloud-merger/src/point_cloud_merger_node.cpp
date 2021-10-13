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