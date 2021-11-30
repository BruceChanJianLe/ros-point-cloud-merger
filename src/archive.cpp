/* 
 * this contains the unit tests from previous implementations
 * 
 * 
 */

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

/* to avoid repetition of setup for each test */
/* class PointCloudMergerTest : public ::testing::Test */
class point_cloud_merger : public ::testing::Test
{
protected:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef sensor_msgs::PointCloud2 PointCloudMsgT;

    typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                            PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                            PointCloudMsgT, PointCloudMsgT>
        SyncPolicyT;

    /* PointCloudMergerTest test()
    {
        // initialisation code
        ros_util::point_cloud_merger merge(true, 1.0, 3.0, 0.5, 2.0, 0.0, 100.0, 2);
    } */

    ros_util::point_cloud_merger merge(bool, double, double, double, double, double, double, int);

    void SetUp()
    {
        // code here will execute just before the test ensues
        merge(true, 1.0, 3.0, 0.5, 2.0, 0.0, 100.0, 2);
    }

    void TearDown()
    {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
    }

    /* void setXMinValue(double x_min_value)
    {
        pmin_range_x_ = x_min_value;
        nmin_range_x_ = -x_min_value;
    } */

public:
    /* PointCloudMergerTest pointcloud(bool, double, double, double, double, double, double, int); */

    std::string input_topics_;
    std::string output_topic_;

    std::string output_frame_id_;

    bool enable_range_flag_;
    bool test_flag_;

    double pmin_range_x_;
    double pmax_range_x_;
    double nmin_range_x_;
    double nmax_range_x_;

    double pmin_range_y_;
    double pmax_range_y_;
    double nmin_range_y_;
    double nmax_range_y_;

    double pmin_range_z_;
    double pmax_range_z_;

    int set_input_size_;
    int input_size_;
};

TEST_F(point_cloud_merger, testReplaceValuesForX)
{
    ros_util::point_cloud_merger merge(true, 1.0, 3.0, 0.5, 2.0, 0.0, 100.0, 2);

    double x_min_value = 1.0;   
    double x_max_value = 3.0;

    merge.setXMinValue(1.0);
    merge.setXMaxValue(3.0);

    EXPECT_EQ(x_min_value, merge.getXMinValue());
    EXPECT_EQ(x_max_value, merge.getXMaxValue());
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

/* APPEND to a specific rosbag */
/* TEST(PointCloudMergerTestCase01, rosbagTesting4)
{ */
    /* rosbag to append to */
    /* std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/extra.bag";
 
     rosbag::Bag bag; */
    /* change Append to Write if want to write instead */
    /* bag.open(bagfile_name, rosbag::bagmode::Append);

    std_msgs::Int32 i;
    i.data = 42; */

    /* numbers: topic, i: message to be added */
    /* bag.write("numbers", ros::Time::now(), i);
    bag.close(); */

    /* Read */
    /* rosbag::Bag bag2;
    bag2.open(bagfile_name, rosbag::bagmode::Read);

    int32_t mess = 0;
    rosbag::View view(bag2); */

    /* strightforward way of checking, check that messages have increse by 1 */
    /* BOOST_FOREACH (rosbag::MessageInstance m, view)
    {
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        mess++;
    }
    EXPECT_EQ(44, mess);
    bag2.close();
} */

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

/* TEST(PointCloudMergerTestCase01, rosbagTesting5)
{
    std::string bagfile_name = "/home/isera2/catkin_ws/src/bagfiles/8ptclouds.bag";
    rosbag::Bag bag;
    bag.open(bagfile_name, rosbag::bagmode::Append);
    bag.close();
} */

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