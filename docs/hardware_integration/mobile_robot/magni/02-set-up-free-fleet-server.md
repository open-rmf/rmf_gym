# Set up Free Fleet Server

Now you will need to set up the Free Fleet Server. This endpoint exists outside of the robot, on the RMF network. Follow [these instructions](https://github.com/open-rmf/free_fleet#server-in-ros2) to set this up on your workspace.

We will need to create a launch file for your server. Copy [this](https://github.com/open-rmf/free_fleet/blob/main/ff_examples_ros2/launch/fake_server.launch.xml) file to your ROS2 package, and modify the following fields:

* Change [the node name](https://github.com/open-rmf/free_fleet/blob/main/ff_examples_ros2/launch/fake_server.launch.xml#L7) to `magni_server_node`
* Change [the fleet name](https://github.com/open-rmf/free_fleet/blob/main/ff_examples_ros2/launch/fake_server.launch.xml#L10) to `tinyRobot` This is important, for RMF to recognize this fleet as part of the `adapters` parameters as seen [here](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/base/base.launch.xml#L29-L33).

We will eventually change [the transform parameters](https://github.com/open-rmf/free_fleet/blob/main/ff_examples_ros2/launch/fake_server.launch.xml#L26-L29) in a later step of this tutorial.
