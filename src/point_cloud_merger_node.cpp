#include "ros-point-cloud-merger/point_cloud_merger.hpp"

const std::string ROSNodeName = "point_cloud_merger_node";

int main(int argc, char ** argv)
{
    ros::init(argc, argv, ROSNodeName);

    return 0;
}