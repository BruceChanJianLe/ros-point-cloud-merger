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
 * 
 * check messages number in rosbag 
 * check correct topic
 */

/* Cannot check if filtered as the merged pointcloud in rosbag is the merged, unfiltered */

#include "rosbag/bag.h"
#include "rosbag/chunked_file.h"
#include "rosbag/view.h"

#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

/* accurate number of messages, size. mode and filename for 8ptclouds.bag */
TEST(PointCloudMergerTestCase01, rosbagTesting1)
{
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8ptclouds.bag";
    std::string link = "https://github.com/strawlab/ros_comm/blob/master/tools/rosbag/testtest_bag.cpp";

    /* Serializes to/from a bag file on disk. */
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Read);

    int32_t message_count = 0;

    /* Specifies a view into a bag file to allow for querying for messages on specific connections withn a time range. */
    rosbag::View view(bag);

    EXPECT_EQ(bag.getSize(), 32960919);
    /* mode write(1), read(2) or append(4) */
    EXPECT_EQ(bag.getMode(), 2);
    /* get filename */
    EXPECT_EQ(bag.getFileName(), "/home/isera2/catkin_ws/src/bagfiles/8ptclouds.bag");

    /* verify the number of message is accurate to the one shown when rosbag info 8pts.bag */
    BOOST_FOREACH (rosbag::MessageInstance m, view)
    {
        /* Templated call to instantiate a message. */
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        message_count++;
    }
    EXPECT_EQ(46, message_count);

    bag.close();
}

/* accurate number of messages, size. mode and filename for 8ptclouds_run.bag*/
TEST(PointCloudMergerTestCase01, rosbagTesting2)
{
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8ptclouds_run.bag";

    /* Serializes to/from a bag file on disk. */
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Read);

    int32_t message_count = 0;

    /* Specifies a view into a bag file to allow for querying for messages on specific connections withn a time range. */
    rosbag::View view(bag);

    EXPECT_EQ(bag.getSize(), 193221351);
    /* mode write(1), read(2) or append(4) */
    EXPECT_EQ(bag.getMode(), 2);
    /* get filename */
    EXPECT_EQ(bag.getFileName(), "/home/isera2/catkin_ws/src/bagfiles/8ptclouds_run.bag");

    /* verify the number of message is accurate to the one shown when rosbag info 8pts.bag */
    BOOST_FOREACH (rosbag::MessageInstance m, view)
    {
        /* Templated call to instantiate a message. */
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        message_count++;
    }
    EXPECT_EQ(298, message_count);

    bag.close();
}

/* XXX check if correct output topic, run rostopic list XXX */
/* TEST(PointCloudMergerTestCase01, rosbagTesting3)
{
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8ptclouds_run.bag";

    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Read);

    int32_t message_count = 0;

    rosbag::View view(bag);

    bag.close();
} */

/* testing for rosbag */
/* rosbag is such that we dont have to run trial */
/* see if output has set and applied to the pointcloud, line 282 */
/* check if pointcloud has successfully merged for the onePOintCloud, ... */

/* append to a specific rosbag */
TEST(PointCloudMergerTestCase01, rosbagTesting4)
{
    /* rosbag to append to */
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/extra.bag";
 
     rosbag::Bag bag;
    /* change Append to Write if want to write instead */
    bag.open(bagfile_name, rosbag::bagmode::Append);

    std_msgs::Int32 i;
    i.data = 42;

    /* numbers: topic, i: message to be added */
    bag.write("numbers", ros::Time::now(), i);
    bag.close();

    /* Read */
    rosbag::Bag bag2;
    bag2.open(bagfile_name, rosbag::bagmode::Read);

    int32_t mess = 0;
    rosbag::View view(bag2);

    /* strightforward way of checking, check that messages have increse by 1 */
    BOOST_FOREACH (rosbag::MessageInstance m, view)
    {
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        mess++;
    }
    EXPECT_EQ(44, mess);
    bag2.close();
}

TEST(PointCloudMergerTestCase01, rosbagTesting5)
{
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8ptclouds.bag";
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Append);
    bag.close();
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

/* with DISABLED -> pass the test without running it */
/* TEST(PointCloudMergerTestCase01, DISABLED_rosbagTestingOld)
{
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8pts.bag";
    std::string link = "https://github.com/strawlab/ros_comm/blob/master/tools/rosbag/testtest_bag.cpp";

    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Read);

    int32_t message_count = 0;

    rosbag::View view(bag);

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
} */