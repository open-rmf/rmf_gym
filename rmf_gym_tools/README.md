# rmf_gym_tools

Tools to help test RMF worlds.

## check_tasks_complete
This process wil loop indefinitely, and terminate once there are no more `active_tasks` when querying the RMF dispatcher.

```
# Check every 5 seconds, and terminate with exit code 1 if we take more than 300 seconds.
ros2 run rmf_gym_tools check_tasks_complete --task_check_period 5.0 --task_check_timeout 300.0

# Check every 10 seconds, and terminate with exit code 1 if we take more than 30 seconds.
ros2 run rmf_gym_tools check_tasks_complete --task_check_period 10.0 --task_check_timeout 30.0
```

## extract_waypoints
Given a path to a building.yaml file, generate a list of waypoint names for each graph that was annotated. 
```
ros2 run rmf_gym_tools extract_waypoints --building_path rmf_gym_worlds/maps/base/base.building.yaml
```

## random_tasks
This script repeatedly issues tasks to a given target fleet.
Currently, only Loop Tasks are available.

```
# First, extract the waypoints from a given building
ros2 run rmf_gym_tools extract_waypoints --building_path rmf_gym_worlds/maps/base/base.building.yaml
# Creates files 0_L1.yml, 0.yml, ...

# Issue random Loops infinitely, using sim time, logging to the folder "logs"
ros2 run rmf_gym_tools random_tasks --task_type loop --task_config_path 0_L1.yml --use_sim_time --log_label logs

# Issue 10 random loops, no sim time. check every 2 seconds, and return with error code 1 if tasks takes more than 200 seconds
ros2 run rmf_gym_tools random_tasks --task_type loop --task_config_path 0_L1.yml --task_count 10 --task_check_period 2.0 --task_check_timeout 200.0
```
