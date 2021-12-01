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
 */

class PointCloudFixture : public ::testing::Test
{
protected:
    /* ros_util::point_cloud_merger pointcloud(bool, double, double, double, double, double, double, int); */
    ros_util::point_cloud_merger pointcloud;

    PointCloudFixture()
    {
        pointcloud.setInputSize(0);
        pointcloud.setXMinValue(0.0);
        pointcloud.setXMaxValue(0.0);
        pointcloud.setYMinValue(0.0);
        pointcloud.setYMaxValue(0.0);
        pointcloud.setZMinValue(0.0);
        pointcloud.setZMaxValue(0.0);
        /* pointcloud(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0); */
    }

    /* void SetUp()
    {
    }

    void TearDown()
    {
    } */
};

TEST_F(PointCloudFixture, replaceXValue)
{
    double x_min_value = 1.0;
    double x_max_value = 3.0;

    pointcloud.setXMinValue(1.0);
    pointcloud.setXMaxValue(3.0);

    EXPECT_EQ(x_min_value, pointcloud.getXMinValue());
    EXPECT_EQ(x_max_value, pointcloud.getXMaxValue());
}

TEST_F(PointCloudFixture, replaceYValue)
{
    double y_min_value = 1.0;
    double y_max_value = 3.0;

    pointcloud.setYMinValue(1.0);
    pointcloud.setYMaxValue(3.0);

    EXPECT_EQ(y_min_value, pointcloud.getYMinValue());
    EXPECT_EQ(y_max_value, pointcloud.getYMaxValue());
}

TEST_F(PointCloudFixture, replaceZValue)
{
    double z_min_value = -10.0;
    double z_max_value = 10.0;

    pointcloud.setZMinValue(-10.0);
    pointcloud.setZMaxValue(10.0);

    EXPECT_EQ(z_min_value, pointcloud.getZMinValue());
    EXPECT_EQ(z_max_value, pointcloud.getZMaxValue());
}

TEST_F(PointCloudFixture, onePointcloud)
{
    bool isPointCloudNumberValid = false;

    pointcloud.setInputSize(1);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, twoPointcloud)
{
    bool isPointCloudNumberValid = true;

    pointcloud.setInputSize(2);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, threePointcloud)
{
    bool isPointCloudNumberValid = true;

    pointcloud.setInputSize(3);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, fourPointcloud)
{
    bool isPointCloudNumberValid = true;

    pointcloud.setInputSize(4);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, fivePointcloud)
{
    bool isPointCloudNumberValid = true;

    pointcloud.setInputSize(5);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, sixPointcloud)
{
    bool isPointCloudNumberValid = true;

    pointcloud.setInputSize(6);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, sevenPointcloud)
{
    bool isPointCloudNumberValid = true;

    pointcloud.setInputSize(7);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, eightPointcloud)
{
    bool isPointCloudNumberValid = true;

    pointcloud.setInputSize(8);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

TEST_F(PointCloudFixture, ninePointcloud)
{
    bool isPointCloudNumberValid = false;

    pointcloud.setInputSize(9);

    EXPECT_EQ(isPointCloudNumberValid, pointcloud.checkInputSize());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "point_cloud_merger_node");
    
    /* ros::AsyncSpinner spinner(1);
    spinner.start();  
    spinner.stop(); */

    ros::shutdown();

    return RUN_ALL_TESTS();
}