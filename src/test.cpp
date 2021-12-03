#include <gtest/gtest.h>
#include <iostream>

#include <boost/filesystem.hpp>

#include <ros-point-cloud-merger/point_cloud_merger.hpp>

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"

#include <string>

#define MAX_SIZE 8
#define MIN_SIZE 2

/* class PointCloudFixture : public ::testing::Test
{
protected:
    PointCloudFixture fixture();

public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef sensor_msgs::PointCloud2 PointCloudMsgT;

    typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                            PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                            PointCloudMsgT, PointCloudMsgT>
        SyncPolicyT;

    std::string input_topics[MAX_SIZE]; */

/* gives the pointclouds standardized naming */
/* std::string namePointClouds()
    {
        std::string input_topics[MAX_SIZE];
        int num = 0;

        for (int i = 0; i < MAX_SIZE; i++)
        {
            input_topics[i] = "/velodyne_points" + std::to_string(++num);
        }

        return input_topics[MAX_SIZE];
    } */

/* 
 * 1. generate at least 9 pointclouds
 * 2. filter pointcloud depending on the input given by the user (starting with 1)
 * 3. merge the pointclouds
 * 4. check if merged pointclouds valid/correct
 * 5. repeat 1-4 for 2 to 9 pointclouds
 */

TEST(TestCaseForMergePointCloud, mergeOnePointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 1);
    ASSERT_FALSE(merge.checkInputSize());
}

TEST(TestCaseForMergePointCloud, mergeTwoPointCloud)
{
    /* 1. check is input size valid */
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 2);
    ASSERT_TRUE(merge.checkInputSize());

    /* 2. generate topics */
    /* std::string input_topics[MAX_SIZE]; */

    /* input_topics[0] = "/velodyne_points";

    for (int i = 1; i < merge.getInputSize(); i++)
    {
        input_topics[i] = "/velodyne_points" + std::to_string(i);
    }

    for (int j = merge.getInputSize(); j < MAX_SIZE; j++)
    {
        input_topics[j] = "/velodyne_points";
    } */

    /* 3. generate pointcloud for topics */

    /* random generator for pointcloud */

    /* 4. generate merged pointcloud */

    /* filter out points */
    /* add all points together */

    /* 5. check if any values out of range */

    /* to double check */
}

TEST(TestCaseForMergePointCloud, mergeThreePointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 3);
    ASSERT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForMergePointCloud, mergeFourPointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 4);
    ASSERT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForMergePointCloud, mergeFivePointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 5);
    ASSERT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForMergePointCloud, mergeSixPointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 6);
    ASSERT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForMergePointCloud, mergeSevenPointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 7);
    ASSERT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForMergePointCloud, mergeEightPointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 8);
    ASSERT_TRUE(merge.checkInputSize());
}

TEST(TestCaseForMergePointCloud, mergeNinePointCloud)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 9);
    ASSERT_FALSE(merge.checkInputSize());
}

/* output set and applied to pointcloud */
TEST(TestForMergePointCloud, setOutput)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    double x_min_value = 1.0;
    double x_max_value = 3.0;
    merge.setXMinValue(1.0);
    merge.setXMaxValue(3.0);
    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());

    double y_min_value = 1.0;
    double y_max_value = 3.0;
    merge.setYMinValue(1.0);
    merge.setYMaxValue(3.0);
    EXPECT_EQ(x_min_value, merge.getYMinValue());
    EXPECT_EQ(x_max_value, merge.getYMaxValue());

    double z_min_value = -10.0;
    double z_max_value = 10.0;
    merge.setZMinValue(-10.0);
    merge.setZMaxValue(10.0);
    EXPECT_EQ(z_min_value, merge.getZMinValue());
    EXPECT_EQ(z_max_value, merge.getZMaxValue());

    merge.setInputSize(2);
    EXPECT_TRUE(merge.checkInputSize());

    /* check output set and applied to pointcloud */
    /* merge.pointcloud_callback(); */
}

/* check if pointcloud successfully merged */
TEST(TestPointCloud, testOnePointCloudSuccessfulMerged)
{
}

TEST(TestCaseForAxis, replaceValues)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    double x_min_value = 1.0;
    double x_max_value = 3.0;

    merge.setXMinValue(1.0);
    merge.setXMaxValue(3.0);

    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());

    double y_min_value = 1.0;
    double y_max_value = 3.0;

    merge.setYMinValue(1.0);
    merge.setYMaxValue(3.0);

    EXPECT_EQ(x_min_value, merge.getYMinValue());
    EXPECT_EQ(x_max_value, merge.getYMaxValue());

    double z_min_value = -10.0;
    double z_max_value = 10.0;

    merge.setZMinValue(-10.0);
    merge.setZMaxValue(10.0);

    EXPECT_EQ(z_min_value, merge.getZMinValue());
    EXPECT_EQ(z_max_value, merge.getZMaxValue());

    merge.setInputSize(2);

    EXPECT_TRUE(merge.checkInputSize());

    /* 
     * value of merge == (true, 1.0, 3.0, 1.0, 3.0, -10.0, 10.0, 2)
     */

    /* launch point_cloud_merger with the values set by test */
}

TEST(TestCaseForAxis, replaceValuesForX)
{
    ros_util::point_cloud_merger merge(true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

    double x_min_value = 1.0;
    double x_max_value = 3.0;

    merge.setXMinValue(1.0);
    merge.setXMaxValue(3.0);

    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());

    /* ROS_INFO_STREAM("Current value of min x value is " << merge.getXMinValue());
    ROS_INFO_STREAM("Current value of max x value is " << merge.getXMaxValue()); */

    /* std::cout << "x min value " << merge.getXMinValue() << std::endl; */

    /* launch point_cloud_merger with the values set by test */
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

    /* ROS_INFO_STREAM("Current value of min y value is " << merge.getYMinValue());
    ROS_INFO_STREAM("Current value of max y value is " << merge.getYMaxValue()); */
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

    /* ROS_INFO_STREAM("Current value of min z value is " << merge.getZMinValue());
    ROS_INFO_STREAM("Current value of max z value is " << merge.getZMaxValue()); */
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