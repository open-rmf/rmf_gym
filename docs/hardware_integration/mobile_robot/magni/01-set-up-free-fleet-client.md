# Set up the Free Fleet Client

RMF operates on fleets of robots, however our robot only exists as an individual robot. We will thus use the [`free_fleet`](https://github.com/open-rmf/free_fleet) as a fleet management system. Free Fleet can act as a bridge between your robot and RMF. Whats more, if your robot uses the ROS1 navigation stack to operate ( like how the Magni is ) then Free Fleet is "almost" plug and play ( some assembly required ).

We should first have a look at the [Free Fleet repository](https://github.com/open-rmf/free_fleet) to get an idea of what it is about.

We should have ROS2 installed on your workstation, and a functioning ROS1 navigation stack on the Magni as described [previously](./00-set-up-robot-navstack.md).

## Preparing the Client on the Magni

As the Magni ships with Ubuntu 16.04, we will have to install a later version of CMake from source due to the dependency on `CycloneDDS`.

On your Magni robot:
```
sudo apt update && sudo apt upgrade
wget https://cmake.org/files/v3.18/cmake-3.18.1.tar.gz
tar -xzvf cmake-3.18.1.tar.gz
cd cmake-3.18.1
./bootstrap
make
sudo make install
```

This process might take quite a while.

Next, follow the instructions [here](https://github.com/open-rmf/free_fleet#client-in-ros1) to build the free fleet client on the Magni. You may want to use `catkin` instead of `colcon` for building the client.

We will need to create a launch file for your client. Place this launch file in your ROS1 package:

```
<launch>
  <node name="free_fleet_client_ros1" pkg="free_fleet_client_ros1" type="free_fleet_client_ros1" output="screen">
    <param name="fleet_name" type="string" value="tinyRobot" />
    <param name="robot_name" command="bash -c &quot;hostname | tr -d '\n' &quot; " />
    <param name="dds_domain" type="int" value="42" />
    <param name="max_dist_to_first_waypoint" type="double" value="10.0" />
    <param name="level_name" type="string" value="L1" />
  </node>
</launch>
```
