#include "ros-point-cloud-merger/point_cloud_merger.hpp"

namespace ros_util
{
    /* 
     * access modifiers: used to create access modifiers within a class
     * e.g., public:
     *         void set_values(int, int);
     * 
     * inheritance: used to specify a base class while creating a child class 
     * e.g., class Child : Base
     * 
     * initialization lists: delimit the beginning of an initialization list
     * e.g., public:
     *         B(int number = 99) : A(number) { }
     * 
     * https://www.quora.com/What-is-the-meaning-of-in-C++
     */

    // Constructor
    point_cloud_merger::point_cloud_merger() : private_nh_(), global_nh_()
    {
        /* param_name – The key to be searched on the parameter server.
           param_val – Storage for the retrieved value.
           default_val – Value to use if the server doesn't contain this parameter. */
        private_nh_.param("input_topics", input_topics_, std::string("[/velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]"));
        private_nh_.param("output_frame_id", output_frame_id_, std::string("/velodyne_frame"));
        private_nh_.param("output_topic_", output_topic_, std::string("/points_concat"));
        private_nh_.param("min_range", min_range, std::string("0.9"));
        private_nh_.param("max_range", max_range, std::string("2.0"));

        YAML::Node topics = YAML::Load(input_topics_);
        input_topics_size_ = topics.size();

        // check range of input topics accepted
        if (input_topics_size_ < 2 || 8 < input_topics_size_)
        {
            ROS_ERROR("The size of input_topics must be between 2 and 8!");
            /* Disconnects everything and unregisters from the master. 
            It is generally not necessary to call this function, as the 
            node will automatically shutdown when all NodeHandles destruct. 
            However, if you want to break out of a spin() loop explicitly, 
            this function allows that. */
            ros::shutdown();
        }

        for (size_t i = 0; i < 8; ++i)
        {
            if (i < input_topics_size_)
            {
                cloud_subscribers_[i] =
                    new message_filters::Subscriber<PointCloudMsgT>(global_nh_, topics[i].as<std::string>(), 1);
            }
            else
            {
                cloud_subscribers_[i] =
                    new message_filters::Subscriber<PointCloudMsgT>(global_nh_, topics[0].as<std::string>(), 1);
            }
        }

        cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
            SyncPolicyT(10), *cloud_subscribers_[0], *cloud_subscribers_[1], *cloud_subscribers_[2], *cloud_subscribers_[3],
            *cloud_subscribers_[4], *cloud_subscribers_[5], *cloud_subscribers_[6], *cloud_subscribers_[7]);

        cloud_synchronizer_->registerCallback(
            boost::bind(&point_cloud_merger::pointcloud_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        cloud_publisher_ = global_nh_.advertise<PointCloudMsgT>(output_topic_, 1);
    }

    void point_cloud_merger::start()
    {
    }

    void point_cloud_merger::pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1, const PointCloudMsgT::ConstPtr &msg2,
                                                 const PointCloudMsgT::ConstPtr &msg3, const PointCloudMsgT::ConstPtr &msg4,
                                                 const PointCloudMsgT::ConstPtr &msg5, const PointCloudMsgT::ConstPtr &msg6,
                                                 const PointCloudMsgT::ConstPtr &msg7, const PointCloudMsgT::ConstPtr &msg8)
    {
        assert(2 <= input_topics_size_ && input_topics_size_ <= 8);

        PointCloudMsgT::ConstPtr msgs[8] = {msg1, msg2, msg3, msg4, msg5, msg6, msg7, msg8};
        PointCloudT::Ptr cloud_sources[8];
        PointCloudT::Ptr cloud_concatenated(new PointCloudT);

        // transform points
        try
        {
            for (size_t i = 0; i < input_topics_size_; ++i)
            {
                // Note: If you use kinetic, you can directly receive messages as
                // PointCloutT.

                /* makeShared(): Copy the cloud to the heap and return a smart pointer
                    Returns:
                    shared pointer to the copy of the cloud */
                cloud_sources[i] = PointCloudT().makeShared();

                /*
                Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object using a field_map.
                Parameters:
                msg – the PCLPointCloud2 binary blob
                cloud – the resultant pcl::PointCloud<T> */
                pcl::fromROSMsg(*msgs[i], *cloud_sources[i]);

                /* Block until a transform is possible or it times out
                Parameters:
                target_frame – The frame into which to transform
                source_frame – The frame from which to transform
                time – The time at which to transform
                timeout – How long to block before failing
                polling_sleep_duration – How often to retest if failed
                error_msg – A pointer to a string which will be filled with why the transform failed, if not NULL */
                tf_listener_.waitForTransform(output_frame_id_, msgs[i]->header.frame_id, ros::Time(0), ros::Duration(1.0));

                /*
                Transforms (Maintain relationship between multiple coordinate frames overtime) a point cloud in a given target TF frame using a TransformListener.
                Parameters:
                target_frame(output_frame_id_) – the target TF frame the point cloud should be transformed to
                target_time(ros::Time(0)) – the target timestamp
                cloud_in(*cloud_sources[i]) – the input point cloud
                fixed_frame(msgs[i]->header.frame_id) – fixed TF frame
                cloud_out(*cloud_sources[i]) – the output point cloud
                tf_listener(tf_listener_) – a TF listener object */
                pcl_ros::transformPointCloud(output_frame_id_, ros::Time(0), *cloud_sources[i], msgs[i]->header.frame_id, *cloud_sources[i], tf_listener_);
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // merge points
        for (size_t i = 0; i < input_topics_size_; ++i)
        {
            *cloud_concatenated += *cloud_sources[i];
        }

        // publish points
        cloud_concatenated->header = pcl_conversions::toPCL(msgs[0]->header);

        cloud_concatenated->header.frame_id = output_frame_id_;

        cloud_publisher_.publish(cloud_concatenated);

    } 

    // Destructor
    point_cloud_merger::~point_cloud_merger()
    {
    }
} // namespace ros_util