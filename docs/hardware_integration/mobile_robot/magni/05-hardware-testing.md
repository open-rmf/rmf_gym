# Hardware Testing

Now comes the cool part. We should be able to use the exact same tests that we run for simulation, to test this hardware world.


## Set Up
On the Magni, launch the free fleet client. For example:
```
roslaunch magni_nav_ros1 free_fleet_client.launch.xml
```

On your workstaion, launch the free fleet server. For example:
```
ros2 launch rmf_gym_worlds magni_free_fleet_server.launch.xml
```

Now, launch the `simple_door` world, without sim time:
```
ros2 launch rmf_gym_worlds simple_door.launch.xml use_sim_time:=false
```

## Run the tests
Run a simple loop test, without using sim time:
```
ros2 launch rmf_gym_worlds test_simple_door_simple_loop.launch.xml use_sim_time:=false task_timeout:=500.0
```

And you will be presented with a glorious sight:
![ff_hardware_test_success](/docs/gifs/hardware_in_the_loop.gif)
