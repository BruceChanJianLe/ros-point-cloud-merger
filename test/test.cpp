#include <gtest/gtest.h>
#include <iostream>

#include <boost/filesystem.hpp>

#include <ros-point-cloud-merger/point_cloud_merger.hpp>
#include <ros-point-cloud-merger/AxisManager.hpp>

/* dummy test */
TEST(TestCase0001, testName0001)
{
    ros_util::AxisValue merge(1, 1, 1, "/points");

    EXPECT_EQ(1, merge.getX());
    EXPECT_EQ(1, merge.getY());
    EXPECT_EQ(1, merge.getZ());
}

/* class MyTestSuite : public ::testing::Test
{
public:
    MyTestSuite() {}
    ~MyTestSuite() {}
}; */

TEST(PointCloudMergerTestCase01, pointCloudMergerTestName01)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 2);

    merge.setXValue(20.0);
    merge.setYValue(20.0);
    merge.setZValue(-10.0);
    merge.setInputSize(1);

    double xValue = 20.0;
    EXPECT_EQ(xValue, merge.getXValue());

    double yValue = 20.0;
    EXPECT_EQ(yValue, merge.getYValue());

    double zValue = -10.0;
    EXPECT_EQ(zValue, merge.getZValue());

    int inputSize = 1;
    EXPECT_EQ(inputSize, merge.getInputSize());

    /* ros_util::AxisValue merge(10.0, 10.0, -1.0, 2);

    merge.setXValue(20.0);
    merge.setYValue(20.0);
    merge.setZValue(-10.0);
    merge.setInputSize(1);

    double xValue = 20.0;
    EXPECT_EQ(xValue, merge.getXValue());

    double yValue = 20.0;
    EXPECT_EQ(yValue, merge.getYValue());

    double zValue = -10.0;
    EXPECT_EQ(zValue, merge.getZValue());

    int inputSize = 1;
    EXPECT_EQ(inputSize, merge.getInputSize()); */
}

TEST(PointCloudMergerTestCase02, pointCloudMergerTestName02)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 1);

    std::string rejected = "Rejected! Out of bound input size.";
    std::string accepted = "Successful!";

    merge.setInputSize(1);
    EXPECT_EQ(rejected, merge.checkInputSize());

    merge.setInputSize(2);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(3);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(4);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(5);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(6);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(7);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(8);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(9);
    EXPECT_EQ(rejected, merge.checkInputSize());

    /* ros_util::AxisValue merge(10.0, 10.0, -1.0, 2);

    std::string rejected = "Rejected! Out of bound input size.";
    std::string accepted = "Successful!";
    
    merge.setInputSize(1);
    EXPECT_EQ(rejected, merge.checkInputSize());

    merge.setInputSize(2);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(3);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(4);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(5);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(6);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(7);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(8);
    EXPECT_EQ(accepted, merge.checkInputSize());

    merge.setInputSize(9);
    EXPECT_EQ(rejected, merge.checkInputSize()); */
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