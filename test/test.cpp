#include <gtest/gtest.h>
#include <iostream>

#include <boost/filesystem.hpp>

#include <ros-point-cloud-merger/point_cloud_merger.hpp>
#include <ros-point-cloud-merger/AxisManager.hpp>

/* dummy test */
/* TEST(TestCase0001, testName0001) {
    AxisValue merge(1, 1, 1, "/points");

    EXPECT_EQ(1, merge.getX());
    EXPECT_EQ(1, merge.getY());
    EXPECT_EQ(1, merge.getZ());
} */

/* class MyTestSuite : public ::testing::Test
{
public:
    MyTestSuite() {}
    ~MyTestSuite() {}
}; */

TEST(MyTestSuite01, testName01)
{   
    /* ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 2); */
    ros_util::point_cloud_merger merge;

    merge.setXValue(10.0);
    merge.setYValue(10.0);
    merge.setZValue(10.0);
    
    double xValue = 10.0;
    EXPECT_EQ(xValue, merge.getXValue());

    double yValue = 10.0;
    EXPECT_EQ(yValue, merge.getYValue());

    double zValue = -1.0;
    EXPECT_EQ(zValue, merge.getZValue());

    int inputSize = 2;
    EXPECT_EQ(inputSize, merge.getInputSize());  
}

TEST(MyTestSuite02, testName02)
{   
    /* ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 1); */

    ros_util::point_cloud_merger merge;
    
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
}

int main(int argc, char **argv)
{
    /* ros::init(argc, argv, "point_cloud_merger_node");
    ros_util::point_cloud_merger node; */

    ::testing::InitGoogleTest(&argc, argv);

    /* ros::spin(); */

    return RUN_ALL_TESTS();
}

/* TEST(TestCase01, testName01)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 2);

    double xValue = 10.0;
    EXPECT_EQ(xValue, merge.getXValue());

    double yValue = 10.0;
    EXPECT_EQ(yValue, merge.getYValue());

    double zValue = -1.0;
    EXPECT_EQ(zValue, merge.getZValue());

    int inputSize = 2;
    EXPECT_EQ(inputSize, merge.getInputSize());  
}

TEST(TestCase001, testName001)
{
    ros_util::point_cloud_merger merge(10.0, 10.0, -1.0, 1);

    std::string rejected = "Rejected! Out of bound input size.";
    std::string accepted = "Successful!";

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
} */

/* TEST(TestCase002, testName002)
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
} */

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