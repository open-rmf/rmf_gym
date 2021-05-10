# Modifying Adapters

We will now modify the adapter launch files for this new `corridorRobot` fleet, from the previous `deliveryRobot` fleet.
```
# Edit  launch/narrow_corridor.launch.xml
For the last group ( for deliveryRobot ):
  Replace "fleet_name" with "corridorRobot"
  Replace deliveryRobot_adapter.launch.xml with corridorRobot_adapter.launch.xml
  Keep "nav_graph_file" as it is, since we are still using Graph 1 ( otherwise you will need to change this graph number correspondingly )
  Modify the robot_prefix of the robot_state_aggregator to "corridotRobot". This state aggregator will look for simulation robots prefixed with [fleet_name] ( in this case corridorRobot ) and add them to the same fleet.

# Edit launch/adapters/corridorRobot_adapter.launch.xml
  Feel free to modify various parameters here, for experimentation
```
