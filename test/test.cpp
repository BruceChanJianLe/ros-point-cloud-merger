#include <gtest/gtest.h>
#include <iostream>

#include <boost/filesystem.hpp>

#include <ros-point-cloud-merger/point_cloud_merger.hpp>
#include <ros-point-cloud-merger/AxisManager.hpp>

/* TESTS TO DO
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
 */

/* make rosbag for 8 pointclouds */

TEST(PointCloudMergerTestCase01, replaceValuesForX)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 0.5, 2.0, 0.0, 100.0, 2);

    double x_min_value = 1.0;
    double x_max_value = 3.0;

    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());
}

TEST(PointCloudMergerTestCase01, replaceValuesForY)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 1.0, 3.0, 0.0, 100.0, 2);

    double y_min_value = 1.0;
    double y_max_value = 3.0;

    EXPECT_EQ(y_min_value, merge.getYMinValue());
    EXPECT_EQ(y_max_value, merge.getYMaxValue());
}

TEST(PointCloudMergerTestCase01, replaceValuesForZ)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 2);

    double z_min_value = -10.0;
    double z_max_value = 10.0;

    EXPECT_EQ(z_min_value, merge.getZMinValue());
    EXPECT_EQ(z_max_value, merge.getZMaxValue());
}

TEST(PointCloudMergerTestCase02, onePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 1);

    bool isPointCloudNumberValid = false;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, twoPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, threePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 3);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, fourPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 4);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, fivePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 5);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, sixPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 6);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, sevenPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 7);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, eightPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 8);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, ninePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 9);

    bool isPointCloudNumberValid = false;

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