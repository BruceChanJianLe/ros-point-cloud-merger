#include <gtest/gtest.h>
#include <iostream>
#include <string>

#include <boost/filesystem.hpp>

#include <ros-point-cloud-merger/point_cloud_merger.hpp>

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"

#define MAX_SIZE 8
#define MIN_SIZE 2

/* 
 * 1. generate at least 9 pointclouds
 * 2. filter pointcloud depending on the input given by the user (starting with 1)
 * 3. merge the pointclouds
 * 4. check if merged pointclouds valid/correct
 * 5. repeat 1-4 for 2 to 9 pointclouds
 */

TEST(TestCaseForAxis, replaceValuesForX)
{
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
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(1);

    EXPECT_FALSE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, twoPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(2);

    EXPECT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, threePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(3);

    EXPECT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, fourPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(4);

    EXPECT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, fivePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(5);

    EXPECT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, sixPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(6);

    EXPECT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, sevenPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(7);

    EXPECT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, eightPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(8);

    EXPECT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForPointCloud, ninePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    merge.setInputSize(9);

    EXPECT_FALSE(merge.checkInputSize());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "point_cloud_merger_node");

    return RUN_ALL_TESTS();
}