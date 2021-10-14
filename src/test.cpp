#include "gtest/gtest.h"

#include "iostream"

#include "ros-point-cloud-merger/point_cloud_merger.hpp"

TEST(TestCase1, testName1)
{
    /* test replace value of x */
    /* CMakeFiles/ros-point-cloud-merger.dir/src/test.cpp.o: In function `TestCase1_testName1_Test::TestBody()':
    test.cpp:(.text+0x2d): undefined reference to `ros_util::point_cloud_merger::point_cloud_merger()'
    test.cpp:(.text+0x52): undefined reference to `ros_util::point_cloud_merger::replaceXValue(double)'
    test.cpp:(.text+0x14b): undefined reference to `ros_util::point_cloud_merger::~point_cloud_merger()'
    test.cpp:(.text+0x1ab): undefined reference to `ros_util::point_cloud_merger::~point_cloud_merger()'
    */
    ros_util::point_cloud_merger merge;

    double output = merge.replaceXValue(10.0);
    double compare = 10.0;

    EXPECT_EQ(compare, output);
}

TEST(TestCase2, testName2)
{
    /* test replace value of y */
    ros_util::point_cloud_merger merge;

    double output = merge.replaceYValue(10.0);
    double compare = 10.0;

    EXPECT_EQ(compare, output);
}

TEST(TestCase3, testName3)
{
    /* test replace value of y */
    ros_util::point_cloud_merger merge;

    double output = merge.replaceZValue(10.0);
    double compare = 10.0;

    EXPECT_EQ(compare, output);
}

TEST(TestCase4, testName4)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 1 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(1);
    std::string compare = "Rejected! Out of bound input size.";

    EXPECT_EQ(compare, output);
}

TEST(TestCase5, testName5)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 2 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(2);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase6, testName6)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 3 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(3);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase7, testName7)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 4 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(4);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase8, testName8)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 5 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(5);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase9, testName9)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 6 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(6);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase10, testName10)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 7 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(7);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase11, testName11)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 8 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(8);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase12, testName12)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 9 */
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(9);
    std::string compare = "Rejected! Out of bound input size.";

    EXPECT_EQ(compare, output);
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "point_cloud_merger_node");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}