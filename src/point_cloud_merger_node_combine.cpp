/* #include "ros-point-cloud-merger/point_cloud_merger.h" */
#include "point_cloud_merger.cpp"

#include <ros/ros.h>

const std::string ROSNodeName = "point_cloud_merger_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROSNodeName);

    ros_util::point_cloud_merger node;

    ros::NodeHandle private_nh_, global_nh_;

    /* launch start function in .hpp which links to .cpp */

    /* node.start(); */

    ros::spin();

    return 0;
}