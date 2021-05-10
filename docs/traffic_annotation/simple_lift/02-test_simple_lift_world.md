# Testing the Simple Lift World

## Modify Test
You should modify `simple_lift_basic_test.launch.xml` with the names of the new waypoints as you have annotated previously, and only issue one set of Loop Requests ( since we only have 1 robot now )


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
ros2 launch rmf_gym_worlds simple_lift_basic_test.launch.xml
```

Congratulations! You have made a new RMF world with a lift.
