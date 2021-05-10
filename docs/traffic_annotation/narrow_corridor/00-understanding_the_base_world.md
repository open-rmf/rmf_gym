# Narrow Corridor - Understanding the Base World

We will first walk through the `base` world and understand what it is, exactly.

First, you should have a look at the structure of launch files in [rmf_demos](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos/launch) and [this_repo](/launch) and understand the general structure.

## Explaining the Base World
There are important components that make up any RMF world. Do follow all the links to get a good understanding.

### Schedule
The traffic schedule is a centralized database of all the intended robot traffic trajectories in a facility. 

* [Launch](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/launch/common.launch.xml#L12-L15)
* [Further Info](https://osrf.github.io/ros2multirobotbook/rmf-core.html#traffic-schedule)

### Blockade Moderator
The Blockade Moderator is necessary for the functioning of Traffic Light Fleet adapter robots.

* [Launch](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/launch/common.launch.xml#L17-L20)
* [Further Info](https://osrf.github.io/ros2multirobotbook/rmf-core.html?highlight=traffic%20light#fleet-adapters)

### Building Map Server
The building map server loads the floor plan of your system.

* [Launch](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/launch/common.launch.xml#L23-L2://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/launch/common.launch.xml#L23-L27)
* [Further Info](https://github.com/open-rmf/rmf_traffic_editor/#rmf_traffic_editor)

### Visualizer
The Visualizer will launch an rviz window with a visualization of RMF operation.
* [Launch](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos/launch/common.launch.xml#L30-L39)
* [Further Info](https://github.com/open-rmf/rmf_visualization)

### The Door / Lift Supervisors
The Door and Lift supervisors exist to act as "adapters" for the infrastructure, ensuring they are not acting on requests that might obstruct an ongoing mobile robot task or accidentally close on it. While this node is running, it maintains the "state" of infrastructure, so any non-RMF attempts to change it ( for example to open a door ) will be negated.

You should always run the supervisors, and publish requests to the `/adapter_door_requests` and `/adapter_lift_requests` topic, ( and not the `lift_requests` and `door_requests` topics ).

* [Launch](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/launch/common.launch.xml#L39-L47)
* [Further Info - Door supervisor](https://osrf.github.io/ros2multirobotbook/integration_doors.html#integration)
* [Further Info - Lift supervisor](https://osrf.github.io/ros2multirobotbook/integration_lifts.html#integration)


### Task Dispatcher
The Task Dispatcher is an implementation of Task bidding and allocation in RMF. When a user submits a new task request, RMF will intelligently assign it to the robot in the fleet that can best perform the task.

* [launch](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos/launch/common.launch.xml#L50-L56)
* [Further Info](https://osrf.github.io/ros2multirobotbook/task.html)
