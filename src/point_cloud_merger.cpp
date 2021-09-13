#include "ros-point-cloud-merger/point_cloud_merger.hpp"

namespace ros_util
{
    // Constructor
    point_cloud_merger::point_cloud_merger()
    {

    }

    // Destructor
    point_cloud_merger::~point_cloud_merger()
    {

    }

    void point_cloud_merger::start()
    {
        ros::spin();
    }

} // namespace ros_util