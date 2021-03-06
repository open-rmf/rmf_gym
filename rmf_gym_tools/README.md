# rmf_gym_tools

Tools to help test RMF worlds.

## envs
This folder contains various options on how to quickly provision a machine to try out RMF. 

## check_environment
This tool does some basic checks to make sure that your current setup is correct.
```
ros2 run rmf_gym_tools check_environment --ros_version foxy --ros_domain_id 0 --dds_version rmw_cyclonedds_cpp
```

## check_tasks_complete
This process wil loop indefinitely, and terminate once there are no more `active_tasks` when querying the RMF dispatcher.

```
# Check every 5 seconds, and terminate with exit code 1 if we take more than 300 seconds.
ros2 run rmf_gym_tools check_tasks_complete --task_check_period 5.0 --task_check_timeout 300.0

# Check every 10 seconds, and terminate with exit code 1 if we take more than 30 seconds.
ros2 run rmf_gym_tools check_tasks_complete --task_check_period 10.0 --task_check_timeout 30.0
```

## extract_waypoints
Given a path to a building.yaml file, generate a list of waypoint names for each graph that was annotated. Generates a bunch of files:
Each Graph will have one generated file with all the waypoints across all levels. Then, more files are also generated that splits the waypoints up by level name.
```
# Extract waypoint names to current folder
ros2 run rmf_gym_tools extract_waypoints --building_path rmf_gym_worlds/maps/base/base.building.yaml --output_path .
```

## get_waypoint_location
Given a path to building.yaml and path to a nav_graph file, and a target waypoint name, we retrieve the physical x, y coordinates, and height of this waypoint.
Useful for spawning robots at specific locations in RMF.
```
ros2 run rmf_gym_tools get_waypoint_location --building_path install/rmf_gym_worlds/share/rmf_gym_worlds/worlds/base/maps/base.building.yaml --nav_graph_path install/rmf_gym_worlds/share/rmf_gym_worlds/maps/base/nav_graphs/0.yaml  --waypoint_name 0_a
```

## spawn_robot_at_waypoint
A helper function that combines `get_waypoint_location` and `spawn_robot` to directly spawn a robot at a waypoint. The config path is the yaml file of (x, y, z) locations generated by the gym_tool `spawn_robot_at_waypoint`.
```
ros2 run rmf_gym_tools spawn_robot_at_waypoint --config_path 0_a.yml --sdf_path install/rmf_demos_assets/share/rmf_demos_assets/models/TinyRobot/model.sdf --robot_name tinyRobot1
```

## spawn_robot
Spawns a given robot to coordinates defined in a yaml file.  The config path is the yaml file of (x, y, z) locations generated by the gym_tool `spawn_robot_at_waypoint`.
```
ros2 run rmf_gym_tools spawn_robot --config_path 0_a.yml --sdf_path install/rmf_demos_assets/share/rmf_demos_assets/models/TinyRobot/model.sdf --robot_name tinyRobot1
```

## despawn_robot
Deletes a robot with a given name
```
ros2 run rmf_gym_tools despawn_robot -m tinyRobot1
```

## dispatch_loop
Sends a Loop Request at the given waypoints.
```
# Send one loop request from 0_a to 0_b that loops exactly once, using sim time
ros2 run rmf_gym_tools dispatch_loop -s 0_a -f 0_b -n 1 --use_sim_time true
```

## random_tasks
This script repeatedly issues tasks to a given target fleet based on a provided yaml file.
Currently, only Loop Tasks are available.

```
# First, extract the waypoints from a given building
ros2 run rmf_gym_tools extract_waypoints --building_path rmf_gym_worlds/worlds/base/maps/base.building.yaml --output_path .
# Creates files 0_L1.yml, 0.yml, ...

# Issue random Loops infinitely, using sim time, logging to the folder "logs"
ros2 run rmf_gym_tools random_tasks --use_sim_time true --task_type loop --task_config_path 0_L1.yml --use_sim_time --log_label logs

# Issue 10 random loops, no sim time. check every 2 seconds, and return with error code 1 if tasks takes more than 200 seconds
ros2 run rmf_gym_tools random_tasks --use_sim_time false --task_type loop --task_config_path 0_L1.yml --task_count 10 --task_check_period 2.0 --task_check_timeout 200.0
```
