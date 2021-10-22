#include "ros-point-cloud-merger/point_cloud_merger.hpp"

#include <ros/ros.h>

#include <iostream>

const std::string ROSNodeName = "point_cloud_merger_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROSNodeName);

    /* no default constructor exists for class "ros_util::point_cloud_merger" */
    /* ros_util::point_cloud_merger node; */

    /* For completion, will be overwritten in gtest */
    /* enabled if running tests */
    bool test_flag = false;

    /* pmin_range_x_ and nmin_range_x_ */
    double x_min = 0.1;
    /* pmax_range_x_ and nmax_range_x_ */
    double x_max = 100.0;

    /* pmin_range_y_ and nmin_range_y_ */
    double y_min = 0.1;
    /* pmax_range_y_ and nmax_range_y_ */
    double y_max = 100.0;

    /* pmin_range_z_ */
    double z_min = 0.1;
    /* pmax_range_z_ */
    double z_max = 100.0;

    /* input size, used in testing, given by users */
    double input_s = 0;
    
    ros_util::point_cloud_merger node(test_flag, x_min, x_max, y_min, y_max, z_min, z_max, input_s);

    ros::spin();

    return 0;
}