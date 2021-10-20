#include "ros-point-cloud-merger/point_cloud_merger.hpp"

#include <ros/ros.h>

#include <iostream>

const std::string ROSNodeName = "point_cloud_merger_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, ROSNodeName);

    /* no default constructor exists for class "ros_util::point_cloud_merger" */
    ros_util::point_cloud_merger node;

    /* std::string input_topics = "[velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]";
    std::string output_topic = "/husky_points_concat";
    std::string output_frame_id = "velodyne_frame";

    double x_min = 0.5;
    double x_max = 2.0;
    double y_min = 0.5;
    double y_max = 2.0;
    double z_min = -1.0;
    double z_max = 50.0;

    int set_input_size = 2; 

    ros_util::point_cloud_merger(input_topics, output_frame_id, output_topic, x_min, x_max, y_min, y_max, z_min, z_max, set_input_size); */

    ros::spin();

    return 0;
}