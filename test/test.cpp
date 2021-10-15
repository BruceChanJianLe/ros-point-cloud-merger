#include "gtest/gtest.h"

#include "iostream"

#include "ros-point-cloud-merger/point_cloud_merger.hpp"

#include "ros-point-cloud-merger/AxisManager.hpp"

/* dummy test */
TEST(TestCase0001, testName0001) {
    AxisValue merge(1, 1, 1, "/points");

    EXPECT_EQ(1, merge.getX());
    EXPECT_EQ(1, merge.getY());
    EXPECT_EQ(1, merge.getZ());
}

/* CMakeFiles/ros-point-cloud-merger.dir/test/test.cpp.o: In function `__static_initialization_and_destruction_0(int, int)':
        test.cpp:(.text+0x16c3): undefined reference to `boost::system::generic_category()'
        test.cpp:(.text+0x16cf): undefined reference to `boost::system::generic_category()'
        test.cpp:(.text+0x16db): undefined reference to `boost::system::system_category()'
        CMakeFiles/ros-point-cloud-merger.dir/test/test.cpp.o: In function `boost::system::error_category::std_category::equivalent(int, std::error_condition const&) const':
        test.cpp:(.text._ZNK5boost6system14error_category12std_category10equivalentEiRKSt15error_condition[_ZNK5boost6system14error_category12std_category10equivalentEiRKSt15error_condition]+0xb8): undefined reference to `boost::system::generic_category()'
        test.cpp:(.text._ZNK5boost6system14error_category12std_category10equivalentEiRKSt15error_condition[_ZNK5boost6system14error_category12std_category10equivalentEiRKSt15error_condition]+0xf3): undefined reference to `boost::system::generic_category()'
        CMakeFiles/ros-point-cloud-merger.dir/test/test.cpp.o: In function `boost::system::error_category::std_category::equivalent(std::error_code const&, int) const':
        test.cpp:(.text._ZNK5boost6system14error_category12std_category10equivalentERKSt10error_codei[_ZNK5boost6system14error_category12std_category10equivalentERKSt10error_codei]+0xb8): undefined reference to `boost::system::generic_category()'
        test.cpp:(.text._ZNK5boost6system14error_category12std_category10equivalentERKSt10error_codei[_ZNK5boost6system14error_category12std_category10equivalentERKSt10error_codei]+0xf3): undefined reference to `boost::system::generic_category()'
        test.cpp:(.text._ZNK5boost6system14error_category12std_category10equivalentERKSt10error_codei[_ZNK5boost6system14error_category12std_category10equivalentERKSt10error_codei]+0x1d2): undefined reference to `boost::system::generic_category()'
        collect2: error: ld returned 1 exit status */


TEST(TestCase01, testName01)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 2);

    double compare = 10.0;

    EXPECT_EQ(compare, merge.getXValue());
    EXPECT_EQ(compare, merge.getYValue());
    EXPECT_EQ(compare, merge.getZValue());
    EXPECT_EQ(compare, merge.getInputSize());
}

TEST(TestCase001, testName001)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 1);

    std::string compare = "Rejected! Out of bound input size.";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase002, testName002)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 2);

    std::string compare = "Successful!";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase003, testName003)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 3);

    std::string compare = "Successful!";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase004, testName004)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 4);

    std::string compare = "Successful!";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase005, testName005)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 5);

    std::string compare = "Successful!";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase006, testName006)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 6);

    std::string compare = "Successful!";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase007, testName007)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 7);

    std::string compare = "Successful!";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase008, testName008)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 8);

    std::string compare = "Successful!";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}

TEST(TestCase009, testName009)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 9);

    std::string compare = "Rejected! Out of bound input size.";
    
    EXPECT_EQ(compare, merge.checkInputSize());
}


/* TEST(TestCase02, testName02)
{
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
} */

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