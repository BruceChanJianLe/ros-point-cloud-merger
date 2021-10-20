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

/* use boolean to validate instead */
/* make rosbag for 8 pointclouds */

TEST(PointCloudMergerTestCase01, replaceValuesForX)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    double x_min_value = 0.5;
    double x_max_value = 5.0;

    merge.setXMinValue(0.5);
    merge.setXMaxValue(5.0);

    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());
}

TEST(PointCloudMergerTestCase01, replaceValuesForY)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    double y_min_value = 0.5;
    double y_max_value = 5.0;

    merge.setYMinValue(0.5);
    merge.setYMaxValue(5.0);

    EXPECT_EQ(y_min_value, merge.getYMinValue());
    EXPECT_EQ(y_max_value, merge.getYMaxValue());
}

TEST(PointCloudMergerTestCase01, replaceValuesForZ)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    double z_min_value = -10.0;
    double z_max_value = 100.0;

    merge.setZMinValue(-10.0);
    merge.setZMaxValue(100.0);

    EXPECT_EQ(z_min_value, merge.getZMinValue());
    EXPECT_EQ(z_max_value, merge.getZMaxValue());
}

TEST(PointCloudMergerTestCase02, onePointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = false;

    merge.setInputSize(1);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, twoPointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(2);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, threePointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(3);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, fourPointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(4);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, fivePointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(5);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, sixPointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(6);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, sevenPointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(7);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, eightPointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    merge.setInputSize(8);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, ninePointCloud)
{
    ros_util::point_cloud_merger merge(10.0, 20.0, 10.0, 20.0, -1.0, 10.0, 2);

    bool isPointCloudNumberValid = false;

    merge.setInputSize(9);

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

/* 
CMakeFiles/test_mynode.dir/test/test.cpp.o: In function `PointCloudMergerTestCase03_newReplaceValuesForX_Test::TestBody()':
test.cpp:(.text+0x1d7d): undefined reference to `ros_util::point_cloud_merger::point_cloud_merger(double, double)'

CMakeFiles/test_mynode.dir/test/test.cpp.o: In function `PointCloudMergerTestCase03_newReplaceValuesForY_Test::TestBody()':
test.cpp:(.text+0x207d): undefined reference to `ros_util::point_cloud_merger::point_cloud_merger(double, double)'
 */

TEST(PointCloudMergerTestCase03, newReplaceValuesForX)
{
    ros_util::point_cloud_merger merge(true, 0.0, 1.0);

    double x_min_value = 0.5;
    double x_max_value = 5.0;

    EXPECT_NE(x_min_value, merge.getXMinValue());
    EXPECT_NE(x_max_value, merge.getXMaxValue());
}

TEST(PointCloudMergerTestCase03, newReplaceValuesForY)
{
    ros_util::point_cloud_merger merge(true, 0.0, 1.0);

    double x_min_value = 0.0;
    double x_max_value = 1.0;

    EXPECT_NE(x_min_value, merge.getXMinValue());
    EXPECT_NE(x_max_value, merge.getXMaxValue());
}

TEST(PointCloudMergerTestCase03, newReplaceValuesForZ)
{
}

TEST(PointCloudMergerTestCase04, newOnePointCloud)
{
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