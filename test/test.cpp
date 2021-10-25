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

/* make rosbag for 8 pointclouds */

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

TEST(PointCloudMergerTestCase01, rosbagTesting)
{
    /* 17 messages */
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8pts.bag";
    std::string link = "https://github.com/strawlab/ros_comm/blob/master/tools/rosbag/testtest_bag.cpp";

    /* Serializes to/from a bag file on disk. */
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Read);

    int32_t message_count = 0;

    /* Specifies a view into a bag file to allow for querying for messages on specific connections withn a time range. */
    rosbag::View view(bag);

    /* 11.6MB = 11.6 * 1024 * 1024 bytes */
    EXPECT_EQ(bag.getSize(), 12187913);
    /* version_ / 100, where version_ == 200 */
    EXPECT_EQ(bag.getMajorVersion(), 2);
    /* version_ % 100, where version_ == 200 */
    EXPECT_EQ(bag.getMinorVersion(), 0);
    /* mode write(1), read(2) or append(4) */
    EXPECT_EQ(bag.getMode(), 2);
    /* get filename */
    EXPECT_EQ(bag.getFileName(), "/home/isera2/catkin_ws/src/bagfiles/8pts.bag");

    /* verify the number of message is accurate to the one shown when rosbag info 8pts.bag */
    BOOST_FOREACH (rosbag::MessageInstance m, view)
    {
        /* Templated call to instantiate a message. */
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        message_count++;
    }
    EXPECT_EQ(17, message_count);

    /*  
     * validate if have pointcloud coming out
     * 
     * can be done in docs/rosbag_run.md
     */

    bag.close();
}

/* with DISABLED -> pass the test without running it */
TEST(PointCloudMergerTestCase01, DISABLED_rosbagTestingOld)
{
    /* NOT SURE WHAT IS GOING ON */
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8pts.bag";
    std::string link = "https://github.com/strawlab/ros_comm/blob/master/tools/rosbag/testtest_bag.cpp";

    /* Serializes to/from a bag file on disk. */
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Read);

    int32_t message_count = 0;

    rosbag::View view(bag);

    /* rosbag record whole poincloud withoutthe restriction on range */
    BOOST_FOREACH (rosbag::MessageInstance m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
        {
            ASSERT_EQ(s->data, std::string("foo"));
            message_count++;
        }
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
        {
            ASSERT_EQ(i->data, 42);
            message_count++;
        }
    }
    bag.close();
    EXPECT_EQ(0, 1);
}

TEST(PointCloudMergerTestCase01, replaceValuesForX)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 0.5, 2.0, 0.0, 100.0, 2);

    double x_min_value = 1.0;
    double x_max_value = 3.0;

    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());
}

TEST(PointCloudMergerTestCase01, replaceValuesForY)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 1.0, 3.0, 0.0, 100.0, 2);

    double y_min_value = 1.0;
    double y_max_value = 3.0;

    EXPECT_EQ(y_min_value, merge.getYMinValue());
    EXPECT_EQ(y_max_value, merge.getYMaxValue());
}

TEST(PointCloudMergerTestCase01, replaceValuesForZ)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 2);

    double z_min_value = -10.0;
    double z_max_value = 10.0;

    EXPECT_EQ(z_min_value, merge.getZMinValue());
    EXPECT_EQ(z_max_value, merge.getZMaxValue());
}

TEST(PointCloudMergerTestCase02, onePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 1);

    bool isPointCloudNumberValid = false;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, twoPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 2);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, threePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 3);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, fourPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 4);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, fivePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 5);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, sixPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 6);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, sevenPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 7);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, eightPointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 8);

    bool isPointCloudNumberValid = true;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
}

TEST(PointCloudMergerTestCase02, ninePointCloud)
{
    ros_util::point_cloud_merger merge(true, 0.5, 2.0, 0.5, 2.0, -10.0, 10.0, 9);

    bool isPointCloudNumberValid = false;

    EXPECT_EQ(isPointCloudNumberValid, merge.checkInputSize());
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