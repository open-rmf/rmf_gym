# Examining the Base World

Now that we understand the components of the Base World, we should try running some code to see things working. First, we should check that our system is configured properly.

```
ros2 run rmf_gym_tools check_environment --ros_version foxy --ros_domain_id 0 --dds_version rmw_cyclonedds_cpp
```

We will launch the `Base World` in full.

```
ros2 launch rmf_gym_worlds base.launch.xml
```

You should see an RViz window pop up. This is the schedule visualizer, a graphical representation of the Schedule.

You should also see Blue "bubbles" overlaying yellow ones. These are the robot vicinities. There are two different fleets, each with one robot. Can you guess how the robot names relate to their fleets?

You should also see purple "bubbles." These are "Fleet States", and represent the actual physical robot location.

We can try issuing tasks to the robots. Remember, the task will be issued to the fleet ( not specific robot ) that is capable.

```
# Lets speed up the simulation
gz physics -u 0

# Send a Loop Task between waypoint 0_a and 0_b. Remember we must use sim_time
ros2 run rmf_demos_tasks dispatch_loop -s 0_a -f 0_b -n 5 --use_sim_time

# Send a Loop Task between waypoint 1_a and 1_b. Remember we must use sim_time
ros2 run rmf_demos_tasks dispatch_loop -s 1_a -f 1_b -n 5 --use_sim_time
```

The robots should start moving in the horizontal and vertical directions. 

You should immediately see green lines projected from the robots. These are trajectories in the schedule that represent the each robots projected space occupation over time, as configured by each fleet adapter's parameters.

You might see the lines turn red. This represents a detection of conflicts, which will then be followed by [negotiation and conflict resolution](https://osrf.github.io/ros2multirobotbook/rmf-core.html?highlight=deconflict#traffic-deconfliction). We should hopefully never see the robots crash into each other or get stuck.

If you want to launch the world without opening graphical panels ( for example in a server ), you can run

```
ros2 launch rmf_gym_worlds base.launch.xml headless:=1
```
