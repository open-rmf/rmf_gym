# rmf_gym_worlds
This repository contains simulation test worlds to help check that common multi-robot scenarios play out correctly.

# Overview
The repository structure is:
```
launch:                                     Contains all launch files
  include:                                  Contains files that are common to many test world test_worlds
    adapters:                               Contains common fleet adapters for all worlds
    rviz:                                   Contains common rviz configurations for all worlds
  tests:                                    Contains all test launch files
    [test_world]                            Contains test launch files for specific test world
      [test_world]_[test_name].launch.xml   Specific test case 
      ...
    ...
  common.launch.xml                         Commonly launch nodes, will always be used
  simulation.launch.xml                     Simulation specific nodes
  [test_world].launch.xml                   Launch file for a specific test world

maps:                                       Contains all building files and floor plans
  [test_world]                              Contains the building files and floor plans for a specific test world
    [test_world].building.yaml              Building file
    [test_world].png                        Floor plan image
```

# The base world
The `Base` world is a template world `10m X 20m` meant to standardize the dimensions of all testing scenarios. The motivation is that
these test worlds can be used to support hardware testing. We can modify the base world template and even add hardware in the loop.

By standardizing the physical space for each test world, it is easier to determine and arrange for hardware tests. 

## Trying the base world
We can already try out the base world and a basic test case.

```
ros2 launch rmf_gym_worlds base.launch.xml
ros2 launch rmf_gym_worlds base_basic_test.launch.xml 
```

We should see the two robots looping in a cross-shape. When 10 Loops have been completed by each robot, the test process will terminate with a `0` exit status.
```
echo $?       # returns 0
```

If the test case fails, the command will return 1. This happens when a timeout is reached. You can use this in our testing infrastructure to carry out contingency measures.

We can also run random loop tasks:
```
ros2 launch rmf_gym_worlds base_random_loops_test.launch.xml
```

# Creating a new Test World
We will now observe how we can modify the base world in order to create a new test world, the `narrow_corridor.` 
The final files are provided for you, but following the below instructions will reproduce the `narrow_corridor` world step by step.
You can also use [this script](./generate_new_world.bash) to carry out all the following steps automatically:
```
NEW_SCENE=narrow_corridor bash generate_new_world.bash
```

## Create Boilerplate Files
We first copy and modify certain files. From the folder of `rmf_gym_worlds`:
```
export NEW_SCENE=narrow_corridor

# This launch file contains all necessary processes for an RMF instance.
cp launch/base.launch.xml launch/$NEW_SCENE.launch.xml
# We then rename base to the new world name
sed -i "s/base/$NEW_SCENE/g" launch/$NEW_SCENE.launch.xml

# This file contains the configuration of the rviz instance we will launch. We can customize it, such as the default floor for visualization
mkdir -p launch/include/rviz/$NEW_SCENE
cp launch/include/rviz/base/base.rviz launch/include/rviz/$NEW_SCENE/$NEW_SCENE.rviz

# These are the files that will modify to represent traffic lanes, and infrastructure like lifts and doors in our world.
mkdir -p maps/$NEW_SCENE
cp -r maps/base/base.building.yaml maps/$NEW_SCENE/$NEW_SCENE.building.yaml
cp -r maps/base/base.png maps/$NEW_SCENE/$NEW_SCENE.png
sed -i "s/base/$NEW_SCENE/g" maps/$NEW_SCENE/$NEW_SCENE.building.yaml

# This is a test file we will use to check a specific testing scenario
# We send two LoopRequests between two corresponding pairs for 10 loops each
mkdir -p launch/tests/$NEW_SCENE
cp launch/tests/base/base_basic_test.launch.xml launch/tests/$NEW_SCENE/$NEW_SCENE\_basic_test.launch.xml

# This is a new adapter we will use for our new fleet
cp launch/include/adapters/deliveryRobot_adapter.launch.xml launch/include/adapters/corridorRobot_adapter.launch.xml
sed -i "s/deliveryRobot/corridorRobot/g" launch/include/adapters/corridorRobot_adapter.launch.xml
```

## Modify Building File
Now we can modify the building file. We will delete a graph, then add a new robot `corridorRobot1` to the fleet `corridotRobot`, and add a door

Open the traffic editor:
```
# Source ROS2 and your workspace
traffic-editor maps/$NEW_SCENE/$NEW_SCENE.building.yaml
```

### Delete Graph 1
There are two Graphs, 0 and 1. You can see this by the two lane colors. Lets remove all annotations for Graph 1.
```
# Look at the top right pane for the word "traffic"
# Select [traffic > Graph 0] and uncheck the Graph 0 checkbox to hide these lanes
# Select [traffic > Graph 1] ( The row should now be highlighted )
# Select the blue vertical bar
# Press Del on your keyboard
# Select the two vertices it was connected to
# For each of these vertices, press Del
# Select [traffic > Graph 0] and uncheck the Graph 0 checkbox to show Graph 0 again
```

### Annotate Graph 1
Now we will reannotate Graph 1. 
```
# Look at the top right pane for the word "traffic"
# Select [traffic > Graph 1] ( The row should now be highlighted )
# Look at the top pane for the word "add lane"
# Select [add lane]. The button text should now turn red
# Draw graph 1 lanes by clicking on points where you want the lane to go. 
# Draw a rectangular loop around the floor, with the bottom of the loop overlapping with the Graph 0 lane.
# Press the move button ( Top pane, cross shaped ) to move individual vertices
# Press the select button ( Top pane, cursor ) to switch back to "normal" mode
# Select each lane you have added, and for each lane, look at the bottom right pane. 
# Modify the "bidirectional" field to true. This will allow the robot to move in both directions on this lane.
# Alternatively, highlight the lane and press "b"
# Name all of these vertices by clicking on them and, on the bottom right pane, fill in the "name" field.
# For one of these graphs, click "add property", select the drop down, and click is_charger.
# For the is_charger row that appeared, change to "true"
```

### Add corridorRobot to Graph 1
We will add a simulation robot to Graph 1 by annotating the traffic lanes.
```
# Select one of the vertices on Graph 1. This will be the vertice where we spawn the simulation DeliveryRobot
# Click on "add property", select spawn_robot_type and fill in "DeliveryRobot". This is the name of the gazebo model as spelled in the sdf file.
# You can have a look at https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_assets/models/DeliveryRobot for reference.
# If you want to add external models, you will have to add the folder to the correct path and use the exact name as specified in the sdf. The simplest is to copy it to `~/.gazebo/models/`. 
# Click on "add property", select spawn_robot_type and fill in "corridorRobot1". The name must be precisely [fleet_name][identifier], so that the simulation code can assign this robot to a corresponding fleet. In this case, this is robot "corridorRobot1" of fleet "corridotRobot".
# For one of these graphs, click "add property", select the drop down, and click is_charger.
# For the is_charger row that appeared, change to "true"
```

### Add Door
We will now add a door to the bottom overlapping lanes of the `narrow_corridor` world.
```
# At the top pane, click the iron with a door icon
# Click two points vertically down the middle of Graph 0. A Door annotation should appear.
# On the bottom right, change "type" to "double_sliding".
# On the bottom right, change "name" to "door1".
```

## Modify Adapters
We will now modify the adapter launch files for this new `corridorRobot` fleet, from the previous `deliveryRobot` fleet.
```
# Edit  launch/$NEW_SCENE.launch.xml
For the last group ( for deliveryRobot ):
  Replace "fleet_name" with "corridorRobot"
  Replace deliveryRobot_adapter.launch.xml with corridorRobot_adapter.launch.xml
  Keep "nav_graph_file" as it is, since we are still using Graph 1 ( otherwise you will need to change this graph number correspondingly )
  Modify the robot_prefix of the robot_state_aggregator to "corridotRobot". This state aggregator will look for simulation robots prefixed with [fleet_name] ( in this case corridorRobot ) and add them to the same fleet.

# Edit launch/adapters/corridorRobot_adapter.launch.xml
  Feel free to modify various parameters here, such as vicinities: https://osrf.github.io/ros2multirobotbook/rmf-core_faq.html?highlight=vicinity#what-distance-is-maintained-between-two-robots
```

## Modify Test
The `base_basic_test.launch.xml` sends two Loop requests between `0_a, 0_b`, and `1_a, 1_b` for 10 loops each. 

The result is that `tinyRobot` fleet, which is on graph 0, loops between `0_a, 0_b`, and the `deliveryRobot` fleet, which is on graph 1, loops between `1_a, 1_b`.

You should modify `$NEW_SCENE_basic_test.launch.xml` with the names of the new waypoints as you have annotated previously.

## Modify Launch File
Finally, we will modify the main launch file. If you have followed all the steps, you will not need to do anything else. However, in other scenarios, you might want to add more fleets on more navigation graphs, or example. In which case, you will need to add / remove FleetAdapter groups:
```
<!-- DeliveryRobot fleet adapter and robot state aggregator needed for the TinyRobot slotcar_plugin -->
<group>
  <let name="fleet_name" value="corridorRobot"/>
  <include file="$(find-pkg-share rmf_demos)/include/adapters/corridorRobot_adapter.launch.xml">
    <arg name="fleet_name" value="$(var fleet_name)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
    <arg name="nav_graph_file" value="$(find-pkg-share rmf_gym_worlds)/maps/narrow_corridor/nav_graphs/1.yaml" />
  </include>
  <include file="$(find-pkg-share rmf_fleet_adapter)/robot_state_aggregator.launch.xml">
    <arg name="robot_prefix" value="corridorRobot"/>
    <arg name="fleet_name" value="$(var fleet_name)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
    <arg name="failover_mode" value="$(var failover_mode)"/>
  </include>
</group>
```

## Rebuild
You can now rebuild your workspace and see the new world in action!
```
colcon build --packages-select rmf_gym_worlds
rm -rf build/rmf_gym_worlds
rm -rf install/rmf_gym_worlds
ros2 launch rmf_gym_worlds $NEW_SCENE.launch.xml
ros2 launch rmf_gym_worlds $NEW_SCENE\_basic_test.launch.xml
```

# Hardware Testing
You can also modify the scenarios above for hardware. You can do that by removing the fleet adapter groups ( such as those that we used for the `corridorRobot` ), and then by launching your own hardware robots fleet adapter instead. Remember to launch without using sim time:
```
ros2 launch rmf_gym_worlds base.launch.xml use_sim_time:=false
```
