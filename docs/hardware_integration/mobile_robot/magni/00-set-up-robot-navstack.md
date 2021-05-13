# Setting up the Robot Navstack

The Magni does not ship with a configured [navigation stack](https://github.com/ros-planning/navigation), so we will need to set one up.

For this experiment we will need to modify the robot base, outfitting it with a LiDAR Sensor.

It is up to you what sensor you want to use. 

Once you have attached a LiDAR to the front of your robot, somewhat like the following image, you are ready to set up the navstack.

You can follow the `README.md` in our [`magni_nav_ros1`](https://github.com/open-rmf/magni_nav_ros1) repository.

We should now have a Magni setup that is able to move autonomously in your workspace using the ROS1 navigation stack.
