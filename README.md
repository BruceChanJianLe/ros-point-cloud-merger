# ROS Point Cloud Merger

This repository demonstrates a simple point cloud merger.

## Error

When using lambda function, you may get error like so
```
 error: no match for call to ‘(std::function<void(const boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >&, const boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >&, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>)>) (const boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >&, const boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >&, const boost::shared_ptr<const message_filters::NullType>&, const boost::shared_ptr<const message_filters::NullType>&, const boost::shared_ptr<const message_filters::NullType>&, const boost::shared_ptr<const message_filters::NullType>&, const boost::shared_ptr<const message_filters::NullType>&, const boost::shared_ptr<const message_filters::NullType>&, const boost::shared_ptr<const message_filters::NullType>&)’
         unwrapper<F>::unwrap(f, 0)(a[base_type::a1_], a[base_type::a2_], a[base_type::a3_], a[base_type::a4_], a[base_type::a5_], a[base_type::a6_], a[base_type::a7_], a[base_type::a8_], a[base_type::a9_]);
         ~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/c++/7/functional:58:0,
                 from /usr/include/boost/smart_ptr/detail/shared_count.hpp:38,
                 from /usr/include/boost/smart_ptr/shared_ptr.hpp:28,
                 from /usr/include/boost/shared_ptr.hpp:17,
                 from /opt/ros/melodic/include/ros/forwards.h:37,
                 from /opt/ros/melodic/include/ros/common.h:37,
                 from /opt/ros/melodic/include/ros/ros.h:43,
                 from /home/chanjl/workspace/point_cloud_merger_handler.hpp:1,
                 from /home/chanjl/workspace/point_cloud_merger_handler.cpp:1:
/usr/include/c++/7/bits/std_function.h:701:5: note: candidate: _Res std::function<_Res(_ArgTypes ...)>::operator()(_ArgTypes ...) const [with _Res = void; _ArgTypes = {const boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >&, const boost::shared_ptr<const sensor_msgs::PointCloud2_<std::allocator<void> > >&, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>, std::shared_ptr<const message_filters::NullType>}]
     function<_Res(_ArgTypes...)>::
     ^~~~~~~~~~~~~~~~~~~~~~~~~~~~
/usr/include/c++/7/bits/std_function.h:701:5: note:   no known conversion for argument 3 from ‘const boost::shared_ptr<const message_filters::NullType>’ to ‘std::shared_ptr<const message_filters::NullType>’
```

Please take note specially, the one for `std_function` as that is the solution to this error.

```cpp
     sync.registerCallback<std::function<void(
                const ImageConstPtr&,
                const CameraInfoConstPtr&,
                const boost::shared_ptr<const message_filters::NullType>,
                const boost::shared_ptr<const message_filters::NullType>,
                const boost::shared_ptr<const message_filters::NullType>,
                const boost::shared_ptr<const message_filters::NullType>,
                const boost::shared_ptr<const message_filters::NullType>,
                const boost::shared_ptr<const message_filters::NullType>,
                const boost::shared_ptr<const message_filters::NullType>
     )>>(
     [](const ImageConstPtr& image_msg, const CameraInfoConstPtr& caminfo_msg, ...)
    { ... }
```

[original_github_question](https://github.com/ros/ros_comm/issues/1803)

## Reference
- [link_rosanswer](https://answers.ros.org/question/346156/merging-point-clouds/)
- [link_example](https://github.com/Autoware-AI/core_perception/blob/master/points_preprocessor/nodes/points_concat_filter/points_concat_filter.cpp)
- [linl_rosanswer2](https://answers.ros.org/question/58626/merging-multiple-pointcloud2/)
