#include "gtest/gtest.h"

#include "iostream"

#include "ros-point-cloud-merger/point_cloud_merger.hpp"

#include "ros-point-cloud-merger/AxisManager.hpp"

TEST(TestCase001, testName001) {
    AxisValue merge(1, 1, 1, "/points");

    EXPECT_EQ(1, merge.getX());
    EXPECT_EQ(1, merge.getY());
    EXPECT_EQ(1, merge.getZ());
}

TEST(TestCase01, testName01)
{
    /* CMakeFiles/ros-point-cloud-merger.dir/test/test.cpp.o: In function `TestCase1_testName1_Test::TestBody()':
        test.cpp:(.text+0x2d): undefined reference to `ros_util::point_cloud_merger::point_cloud_merger()'
        test.cpp:(.text+0x52): undefined reference to `ros_util::point_cloud_merger::replaceXValue(double)'
        test.cpp:(.text+0x161): undefined reference to `ros_util::point_cloud_merger::replaceYValue(double)'
        test.cpp:(.text+0x270): undefined reference to `ros_util::point_cloud_merger::replaceZValue(double)'
        test.cpp:(.text+0x369): undefined reference to `ros_util::point_cloud_merger::~point_cloud_merger()'
        test.cpp:(.text+0x448): undefined reference to `ros_util::point_cloud_merger::~point_cloud_merger()' */

    ros_util::point_cloud_merger merge;

    double output, compare;

    output = merge.replaceXValue(10.0);
    compare = 10.0;
    EXPECT_EQ(compare, output);

    output = merge.replaceYValue(10.0);
    compare = 10.0;
    EXPECT_EQ(compare, output);

    output = merge.replaceZValue(-1.0);
    compare = -1.0;
    EXPECT_EQ(compare, output);
}

TEST(TestCase02, testName02)
{
    /* CMakeFiles/ros-point-cloud-merger.dir/test/test.cpp.o: In function `TestCase02_testName02_Test::TestBody()':
        test.cpp:(.text+0x493): undefined reference to `ros_util::point_cloud_merger::point_cloud_merger()'
        test.cpp:(.text+0x4cf): undefined reference to `ros_util::point_cloud_merger::checkInputSize[abi:cxx11](int)'
        test.cpp:(.text+0x5f9): undefined reference to `ros_util::point_cloud_merger::checkInputSize[abi:cxx11](int)'
        test.cpp:(.text+0x723): undefined reference to `ros_util::point_cloud_merger::checkInputSize[abi:cxx11](int)'
        test.cpp:(.text+0x84d): undefined reference to `ros_util::point_cloud_merger::checkInputSize[abi:cxx11](int)'
        test.cpp:(.text+0x977): undefined reference to `ros_util::point_cloud_merger::checkInputSize[abi:cxx11](int)'
        CMakeFiles/ros-point-cloud-merger.dir/test/test.cpp.o:test.cpp:(.text+0xaa1): more undefined references to `ros_util::point_cloud_merger::checkInputSize[abi:cxx11](int)' follow
        CMakeFiles/ros-point-cloud-merger.dir/test/test.cpp.o: In function `TestCase02_testName02_Test::TestBody()':
        test.cpp:(.text+0xf58): undefined reference to `ros_util::point_cloud_merger::~point_cloud_merger()'
        test.cpp:(.text+0x11cf): undefined reference to `ros_util::point_cloud_merger::~point_cloud_merger()' */

    ros_util::point_cloud_merger merge;

    std::string output, compare;

    output = merge.checkInputSize(1);
    compare = "Rejected! Out of bound input size.";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(2);
    compare = "Successful!";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(3);
    compare = "Successful!";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(4);
    compare = "Successful!";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(5);
    compare = "Successful!";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(6);
    compare = "Successful!";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(7);
    compare = "Successful!";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(8);
    compare = "Successful!";
    EXPECT_EQ(compare, output);

    output = merge.checkInputSize(9);
    compare = "Rejected! Out of bound input size.";
    EXPECT_EQ(compare, output);
}

/* TEST(TestCase1, testName1)
{
    ros_util::point_cloud_merger merge;

    double output = merge.replaceXValue(10.0);
    double compare = 10.0;

    EXPECT_EQ(compare, output);
}

TEST(TestCase2, testName2)
{
    ros_util::point_cloud_merger merge;

    double output = merge.replaceYValue(10.0);
    double compare = 10.0;

    EXPECT_EQ(compare, output);
}

TEST(TestCase3, testName3)
{
    ros_util::point_cloud_merger merge;

    double output = merge.replaceZValue(-1.0);
    double compare = -1.0;

    EXPECT_EQ(compare, output);
} */

/* TEST(TestCase4, testName4)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(1);
    std::string compare = "Rejected! Out of bound input size.";

    EXPECT_EQ(compare, output);
}

TEST(TestCase5, testName5)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(2);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase6, testName6)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(3);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase7, testName7)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(4);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase8, testName8)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(5);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase9, testName9)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(6);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase10, testName10)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(7);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase11, testName11)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(8);
    std::string compare = "Successful!";

    EXPECT_EQ(compare, output);
}

TEST(TestCase12, testName12)
{
    ros_util::point_cloud_merger merge;

    std::string output = merge.checkInputSize(9);
    std::string compare = "Rejected! Out of bound input size.";

    EXPECT_EQ(compare, output);
} */

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

    /* ros::init(argc, argv, "point_cloud_merger_node");
    ros::NodeHandle nh; */
    /* ros::NodeHandle private_nh("~"); */

    /* tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener_(tfBuffer); */

    /* ros::spin(); */

    return RUN_ALL_TESTS();
}