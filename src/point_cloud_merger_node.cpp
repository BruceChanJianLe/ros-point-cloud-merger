#include "ros-point-cloud-merger/point_cloud_merger.hpp"

const std::string ROSNodeName = "point_cloud_merger_node";


// launch node for points_concat_filter.cpp
int main(int argc, char ** argv)
{
    ros::init(argc, argv, ROSNodeName);

    ros_util::point_cloud_merger node;

    node.start();

    return 0;
}