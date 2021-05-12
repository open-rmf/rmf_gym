# Testing the Simple Lift World

## Modify Test
You should modify `test_simple_lift_simple_loop_sim.launch.xml` with the names of the new waypoints as you have annotated previously, and only issue one set of Loop Requests ( since we only have 1 robot now )

## Modify Robot Start Point
You could modify `simple_lift_sim_setup.launch.xml` with the waypoint you would like the robot to start from, especially if you have renamed waypoints.

## Rebuild and Run
You can now rebuild your workspace and see the new world in action!

Check environment:

```
ros2 run rmf_gym_tools check_environment --ros_version foxy --ros_domain_id 0 --dds_version rmw_cyclonedds_cpp
```

Lets go!
```
rm -rf build/rmf_gym_worlds
rm -rf install/rmf_gym_worlds
colcon build --packages-select rmf_gym_worlds

ros2 launch rmf_gym_worlds simple_lift.launch.xml
ros2 launch rmf_gym_worlds simple_lift_sim_setup.launch.xml
ros2 launch rmf_gym_worlds test_simple_lift_simple_loop.launch.xml use_sim_time:=true
```

Congratulations! You have made a new RMF world with a lift.
