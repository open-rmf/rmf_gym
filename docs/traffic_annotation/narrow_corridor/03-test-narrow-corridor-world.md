# Testing the Narrow Corridor World

## Modify Test
The `base_basic_test.launch.xml` sends two Loop requests between `0_a, 0_b`, and `1_a, 1_b` for 10 loops each. 

The result is that `tinyRobot` fleet, which is on graph 0, loops between `0_a, 0_b`, and the `deliveryRobot` fleet, which is on graph 1, loops between `1_a, 1_b`.

You should modify `narrow_corridor_basic_test.launch.xml` with the names of the new waypoints as you have annotated previously.


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

ros2 launch rmf_gym_worlds narrow_corridor.launch.xml
ros2 launch rmf_gym_worlds narrow_corridor_basic_test.launch.xml
```

Congratulations! You have made a new RMF world. 
