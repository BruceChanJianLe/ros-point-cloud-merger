#include "ros-point-cloud-merger/point_cloud_merger.hpp"
#include <ros/ros.h>

const std::string ROSNodeName = "point_cloud_merger";

int main(int argc, char ** argv)
{
    ros::init(argc, argv, ROSNodeName);

    ros_util::point_cloud_merger node;

    /* launch start function in .hpp which links to .cpp */
    /* node.start(); */

    ros::spin();

    return 0;
}