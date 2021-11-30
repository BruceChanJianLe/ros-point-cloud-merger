#include <gtest/gtest.h>
#include <iostream>

#include <boost/filesystem.hpp>

#include <ros-point-cloud-merger/point_cloud_merger.hpp>
#include <ros-point-cloud-merger/AxisManager.hpp>

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

/* TESTS TO DO (alter actual code)
 * 
 * maybe have function?
 * replace value for x -> pass
 * replace value for y -> pass
 * replace value for z -> pass
 * 
 * use assert avilable of create a new function
 * merge of a single point cloud -> fail
 * merge of two point clouds -> pass
 * merge of three point clouds -> pass
 * merge of four point clouds -> pass
 * merge of five point clouds -> pass
 * merge of six point clouds -> pass
 * merge of seven point clouds -> pass
 * merge of eight point clouds -> pass
 * merge of nine point clouds ->fail
 * 
 */

/* Cannot check if filtered as the merged pointcloud in rosbag is the merged, unfiltered */

/* 
 * testing for rosbag 
 * 1. dont have to run trial
 * 2. output has set and applied to the pointcloud
 * 3. check if pointcloud has successfully merged for the onePointCloud, twoPointCloud, etc...
 * 
 */

TEST(TestCaseForAxis, replaceValuesForX)
{
    /* initialisation */
    /* ros_util::point_cloud_merger merge(true, 1.0, 3.0, 0.5, 2.0, 0.0, 100.0, 2); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    double x_min_value = 1.0;
    double x_max_value = 3.0;

    merge.setXMinValue(1.0);
    merge.setXMaxValue(3.0);

    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());
}

TEST(TestCaseForAxis, replaceValuesForY)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 1.0, 3.0, 0.0, 100.0, 2); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    double y_min_value = 1.0;
    double y_max_value = 3.0;

    merge.setYMinValue(1.0);
    merge.setYMaxValue(3.0);

    EXPECT_EQ(y_min_value, merge.getYMinValue());
    EXPECT_EQ(y_max_value, merge.getYMaxValue());
}

TEST(TestCaseForAxis, replaceValuesForZ)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 2); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    double z_min_value = -10.0;
    double z_max_value = 10.0;

    merge.setZMinValue(-10.0);
    merge.setZMaxValue(10.0);

    EXPECT_EQ(z_min_value, merge.getZMinValue());
    EXPECT_EQ(z_max_value, merge.getZMaxValue());
}

TEST(TestCaseForPointCloud, onePointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 1); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = false;

    merge.setInputSize(1);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, twoPointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 2); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(2);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, threePointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 3); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(3);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, fourPointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 4); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(4);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, fivePointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 5); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(5);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, sixPointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 6); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(6);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, sevenPointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 7); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(7);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, eightPointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 8); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(8);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(TestCaseForPointCloud, ninePointCloud)
{
    /* ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 9); */
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    bool isPointCloudNumberValid = false;

    merge.setInputSize(9);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "point_cloud_merger_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();

    return ret;
}