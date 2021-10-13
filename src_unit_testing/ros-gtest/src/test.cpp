#include "gtest/gtest.h"

#include "iostream"

#include "ros-gtest/AxisManager.hpp"
/* #include "ros-gtest/PointCloudMerger.hpp" */
/* #include "ros-point-cloud-merger/point_cloud_merger.hpp" */

TEST(TestCase00, testName00)
{
    AxisValue value(4, 5, 6, "/points");

    ASSERT_NE(5, value.getX());
    ASSERT_EQ(4, value.getX());
    ASSERT_NE(4, value.getY());
    ASSERT_NE(4, value.getZ());
}

TEST(TestCase01, testName01)
{
    AxisValue value(4, 5, 6, "/points");
    std::string compare = "/points";

    ASSERT_EQ(compare, value.getInputTopics());
}

TEST(TestCase02, testName02)
{
    AxisValue value(4, 5, 6, "/points");
    std::string compare = "/POINTS";

    EXPECT_NE(compare, value.getInputTopics());
}

TEST(TestCase1, testName1)
{
    /* test replace value of x */
    /* ros_util::point_cloud_merger merge();
    
    double output = ros_util::point_cloud_merger::replaceXValue(10.0);
    double compare = 10.0;

    EXPECT_EQ(compare, output); */
    
}

TEST(TestCase2, testName2)
{
    /* test replace value of y */
}

TEST(TestCase3, testName3)
{
    /* test replace value of y */
}

TEST(TestCase4, testName4)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 1 */
}

TEST(TestCase5, testName5)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 2 */
}

TEST(TestCase6, testName6)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 3 */
}

TEST(TestCase7, testName7)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 4 */
}

TEST(TestCase8, testName8)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 5 */
}

TEST(TestCase9, testName9)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 6 */
}

TEST(TestCase10, testName10)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 7 */
}

TEST(TestCase11, testName11)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 8 */
}

TEST(TestCase12, testName12)
{
    /* test input_size, method checkInputSize(int) */
    /* input_size == 9 */
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

    /* ros::init(argc, argv, "tester");
    ros::NodeHandle nh; */

    return RUN_ALL_TESTS();
}