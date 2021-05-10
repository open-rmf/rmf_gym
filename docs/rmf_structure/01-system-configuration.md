# System Configuration

There are many ways to configure RMF due to its multi-system nature. This gym is currently tested working with:

* Ubuntu 20.04
* Gazebo 11
* Following the `rmf_demos` [source build instructions](https://github.com/open-rmf/rmf)
* `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
* `ROS_DOMAIN_ID=0`
* No CycloneDDS configuration file

You can check your setup matches this description:
```
ros2 run rmf_gym_tools check_environment --ros_version foxy --ros_domain_id 0 --dds_version rmw_cyclonedds_cpp
```

## Variations

### Binary Install
You might want to install RMF binaries instead of building from source. Since this repository is aimed at testing RMF, we target the latest build from source. The instructions [here](https://osrf.github.io/ros2multirobotbook/intro.html?highlight=RMF#setup-sources-and-installation-of-rmf) describe how to install RMF from binaries.

### DDS Implementations
There are various DDS implementations you can use. IT is important, however, that all machines on your ROS2 network use the same DDS implementation, to prevent nasty errors.

### Different Domain ID
You can segment different ROS2 networks by specifying the domain id. Only nodes on the same domain ID are able to communicate. You must make sure all devices which should be able to communicate are on the same domain ID.

### Routers and NAT
We test all simulations on a simple device setup, meaning no other devices are communicated with. To expand RMF to multi device systems, you will have to connect everything on the same network. It is recommended to use a simple, flat structure to start experimenting, before moving to more complicated network topologies.

It is known that DDS does not play well across routers. It is best not to use DDS across routers, without some sort of detailed networking knowledge, or a VPN.
