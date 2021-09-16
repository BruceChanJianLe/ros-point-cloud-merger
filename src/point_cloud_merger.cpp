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
        /* param function
           Parameters:
           param_name – The key to be searched on the parameter server.
           param_val – Storage for the retrieved value.
           default_val – Value to use if the server doesn't contain this parameter. */
        private_nh_.param("input_topics", input_topics_, std::string("[/velodyne_points, /velodyne_points1, /velodyne_points2, /velodyne_points3, /velodyne_points4, /velodyne_points5, /velodyne_points6, /velodyne_points7]"));
        private_nh_.param("output_frame_id", output_frame_id_, std::string("/velodyne_frame"));
        private_nh_.param("output_topic_", output_topic_, std::string("/points_concat"));
        private_nh_.param("min_range", min_range, std::string("0.9"));
        private_nh_.param("max_range", max_range, std::string("2.0"));

        /* namespace YAML, class Node in library yaml-cpp */
        /* YAML::Node YAML::Load(const std::string &input) */
        YAML::Node topics = YAML::Load(input_topics_);

        /* cannot input_topics_.size() */
        input_topics_size_ = topics.size();

        // check range of input topics accepted
        if (input_topics_size_ < 2 || input_topics_size_ > 8)
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

            }
            else
            {
                
            }
        }
    }

    void point_cloud_merger::start()
    {
    }

    

    // Destructor
    point_cloud_merger::~point_cloud_merger()
    {
    }
} // namespace ros_util