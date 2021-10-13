#include "ros-point-cloud-merger/points_concat_filter.hpp"
#include <ros/ros.h>

const std::string ROSNodeName = "points_concat_filter";

// launch node for points_concat_filter.cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, ROSNodeName);

    PointsConcatFilter node;

    ros::spin();

    return 0;
}