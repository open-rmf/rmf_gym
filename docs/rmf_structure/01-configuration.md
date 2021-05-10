# Configuration

There are many ways to configure RMF due to its multi-system nature. This gym is currently tested working with:

* Ubuntu 20.04
* Gazebo 11
* `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
* `ROS_DOMAIN_ID=0`
* No CycloneDDS configuration file

You can check your setup matches this description:
```
ros2 run rmf_gym_tools check_environment --ros_version foxy --ros_domain_id 0 --dds_version rmw_cyclonedds_cpp
```
