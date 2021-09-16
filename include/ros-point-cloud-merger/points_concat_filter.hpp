#ifndef ROS_POINTS_CONCAT_FILTER_H__
#define ROS_POINTS_CONCAT_FILTER_H__

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <velodyne_pointcloud/point_types.h>

#include <yaml-cpp/yaml.h>

class PointsConcatFilter
{
public:
  PointsConcatFilter();

private:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::PointCloud2 PointCloudMsgT;

  /* ApproximateTime: match messages even if they have different time stamps */
  typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT, PointCloudMsgT,
                                                          PointCloudMsgT, PointCloudMsgT>
      SyncPolicyT;

  ros::NodeHandle node_handle_, private_node_handle_;

  /* Manages an advertisement on a specific topic. A Publisher should always be created through a call to 
        NodeHandle::advertise(), or copied from one that was. Once all copies of a specific Publisher go out of 
        scope, any subscriber status callbacks associated with that handle will stop being called. Once all 
        Publishers for a given topic go out of scope the topic will be unadvertised. */
  message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[8];

  /* typedef */
  message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;

  /* Manages an subscription callback on a specific topic. A Subscriber should always be created 
            through a call to NodeHandle::subscribe(), or copied from one that was. Once all copies of 
            a specific Subscriber go out of scope, the subscription callback associated with that 
            handle will stop being called. Once all Subscriber for a given topic go out of scope the 
            topic will be unsubscribed. */
  ros::Subscriber config_subscriber_;

  /* Manages an advertisement on a specific topic. A Publisher should always be created through 
        a call to NodeHandle::advertise(), or copied from one that was. Once all copies of a specific 
        Publisher go out of scope, any subscriber status callbacks associated with that handle will 
        stop being called. Once all Publishers for a given topic go out of scope the topic will 
        be unadvertised. */
  ros::Publisher cloud_publisher_;

  /* subscribes to message and automatically stores incoming data */
  tf::TransformListener tf_listener_;

  size_t input_topics_size_;

  std::string input_topics_;

  std::string output_frame_id_;

  std::string min_range;
  std::string max_range;

  /* void pointcloud_callback(PointCloudMsgT::Ptr &msg1, PointCloudMsgT::Ptr &msg2,
                           PointCloudMsgT::Ptr &msg3, PointCloudMsgT::Ptr &msg4,
                           PointCloudMsgT::Ptr &msg5, PointCloudMsgT::Ptr &msg6,
                           PointCloudMsgT::Ptr &msg7, PointCloudMsgT::Ptr &msg8); */
  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                           const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                           const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                           const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8);
};

#endif