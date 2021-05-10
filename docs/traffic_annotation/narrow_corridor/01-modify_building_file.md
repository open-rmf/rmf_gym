# Modify Building File for Narrow Corridor World
Here we show, step by step, how to modify the Base World into the Narrow Corridor World.

Open the traffic editor:
```
# Source ROS2 and your workspace
traffic-editor maps/narrow_corridor/narrow_corridor.building.yaml
```

We will be using the [Traffic Editor Secion](https://osrf.github.io/ros2multirobotbook/traffic-editor.html) for reference.

Always make sure that your graphs are fully connected, and not just look like it. A common mistake is having two overlapping vertices, instead of one vertex joining two edges:
```
A-B B'-C      # bad
A-B-C         # Good
```

### Delete Graph 1
There are two Graphs, 0 and 1. You can see this by the two lane colors. Lets remove all annotations for Graph 1.
```
# Look at sidebar for the word "traffic"
# Select [traffic > Graph 0] and uncheck the Graph 0 checkbox to hide these lanes
# Select [traffic > Graph 1] ( The row should now be highlighted )
# Select the Graph 1 lane ( blue vertical bar )
# Press Del on your keyboard
# Select the two vertices it was connected to
# For each of these vertices, press Del
# Select [traffic > Graph 0] and uncheck the Graph 0 checkbox to show Graph 0 again
```

### Annotate Graph 1
Now we will reannotate Graph 1. 

Read how to annotate a [vertex](https://osrf.github.io/ros2multirobotbook/traffic-editor.html#adding-a-vertex) and [lane](https://osrf.github.io/ros2multirobotbook/traffic-editor.html#adding-a-traffic-lane). You should also know how to [Select and Move vertices](https://osrf.github.io/ros2multirobotbook/traffic-editor.html#gui-layout).
```
# Look at sidebar for the word "traffic"
# Select [traffic > Graph 1] ( The row should now be highlighted )
# Select [add lane] on the toolbar. The button text should now turn red
# Draw a rectangular loop around the floor, with the bottom of the loop overlapping with the Graph 0 lane.
# Select each lane you have added, and for each lane, look at the sidebar. 
# Modify the "bidirectional" field to true. This will allow the robot to move in both directions on this lane.
# Alternatively, highlight the lane and press "b"
# Name all of these vertices uniquely.
# Set one of these vertices to be a charger.
```

### Add corridorRobot to Graph 1
We will add a simulation robot to Graph 1 by annotating the traffic lanes.
```
# Select one of the vertices on Graph 1. This will be the vertice where we spawn the simulation DeliveryRobot
# Click on "add property", select spawn_robot_type and fill in "DeliveryRobot". This is the name of the gazebo model as spelled in the sdf file.
# You can have a look at https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_assets/models/DeliveryRobot for reference.
# If you want to add external models, you will have to add the folder to the correct place and use the exact name as specified in the sdf. The simplest place is to copy it to `~/.gazebo/models/`. 
# Click on "add property", select spawn_robot_type and fill in "corridorRobot1". The name must be precisely [fleet_name][identifier], so that the simulation code can assign this robot to a corresponding fleet. In this case, this is robot "corridorRobot1" of fleet "corridotRobot".
```

### Add Door
We will now add a door to the bottom overlapping lanes of the `narrow_corridor` world.

Read how to [annotate doors](https://osrf.github.io/ros2multirobotbook/traffic-editor.html#adding-a-door).
```
# At the top pane, click the iron with a door icon
# Click two points vertically down the middle of Graph 0. A Door annotation should appear.
# On the sidebar, change "type" to "double_sliding".
# On the sidebar, change "name" to "door1".
```
