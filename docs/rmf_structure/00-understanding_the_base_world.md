# Understanding the Base World

We will first walk through the `base` world and understand what it is, exactly.

First, you should have a look at the structure of launch files in [rmf_demos](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos/launch) and the [Base World](/rmf_gym_worlds/worlds/base) and understand the general structure.

## Explaining the Base World
There are important components that make up any RMF world. Do follow all the links to get a good understanding.

A World is put together by launching a variety of processes, each doing an important task, as described below. 

For using the Base world ( and others ) with real hardware, see [TODO].

### Simulation Time
It is important to note that for simulations, we use simulation time. Once hardware is added to the mix, we have to configure our systems accordingly. The `use_sim_time` parameter is crucial to make this distinction. Nodes should be consistently either using sim time, or not.

### Schedule
The traffic schedule is a centralized database of all the intended robot traffic trajectories in a facility. 

* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/common.launch.xml#L12-L15)
* [Further Info](https://osrf.github.io/ros2multirobotbook/rmf-core.html#traffic-schedule)

### Blockade Moderator
The Blockade Moderator is necessary for the functioning of Traffic Light Fleet adapter robots.

* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/common.launch.xml#L17-L20)
* [Further Info](https://osrf.github.io/ros2multirobotbook/rmf-core.html?highlight=traffic%20light#fleet-adapters)

### Building Map Server
The building map server "serves" the floor plan of your system.

* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/common.launch.xml#L23-L27)
* [Further Info](https://github.com/open-rmf/rmf_traffic_editor/#rmf_traffic_editor)

### Visualizer
The Visualizer will launch an rviz window with a visualization of RMF operation.
* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/common.launch.xml#L29-L37)
* [Further Info](https://github.com/open-rmf/rmf_visualization)

### The Door / Lift Supervisors
The Door and Lift supervisors exist to act as "adapters" for the infrastructure, ensuring they are not acting on requests that might obstruct an ongoing mobile robot task or accidentally close on it. While this node is running, it maintains the "state" of infrastructure, so any non-RMF attempts to change it ( for example to open a door ) will be negated.

You should always run the supervisors, and publish requests to the `/adapter_door_requests` and `/adapter_lift_requests` topic, ( and not the `lift_requests` and `door_requests` topics ).

* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/common.launch.xml#L39-L47)
* [Further Info - Door supervisor](https://osrf.github.io/ros2multirobotbook/integration_doors.html#integration)
* [Further Info - Lift supervisor](https://osrf.github.io/ros2multirobotbook/integration_lifts.html#integration)


### Task Dispatcher
The Task Dispatcher is an implementation of Task bidding and allocation in RMF. When a user submits a new task request, RMF will intelligently assign it to the fleet that can best perform the task.

* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/common.launch.xml#L49-L56)
* [Further Info](https://osrf.github.io/ros2multirobotbook/task.html)


### Simulation
The simulation launch file describes how to set up the simulation components. The simulation engines are Gazebo and Ignition Gazebo. We will use simulation extensively in the gym to simplify the learning and testing process.

We use the `slotcar` plugin to simulate robots in RMF. This plugin is "attached" to a Gazebo model, and simulates the physics to move the model.

It is a simple plugin meant to approximate a robot navigation stack. Its obstacle "avoidance" is based on object names. It is thus important to name the simulation entities correctly, so that the robot can do "obstacle detection". In particular, a lift in simulation must have the word "lift" in its name, a door in simulation must have the word "door".
* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/simulation.launch.xml)
* [Further Info](https://osrf.github.io/ros2multirobotbook/simulation.html)
* [SlotCar](https://github.com/open-rmf/rmf_simulation/blob/main/rmf_robot_sim_gazebo_plugins/src/slotcar.cpp)
* [SlotCar code for obstacle detection](https://github.com/open-rmf/rmf_simulation/blob/main/rmf_robot_sim_gazebo_plugins/src/slotcar.cpp#L107-L127)
* [TinyRobot SDF](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_assets/models/TinyRobot/model.sd://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_assets/models/TinyRobot/model.sdf)

### Robot State Aggregators
For simulations, we spawn individual robots. RMF interacts at a fleet level ( I.E, a group of robots ).  A robot state aggregator
is used in simulation settings by aggregating simulation robots into fleets. For the Robot State Aggregator to work, each invidual robot must be correctly named, by prefixing the fleet name.

Thus, if a robot state aggregator `robot_prefix` is set to "tinyRobot", then all robots in the simulation with their named prefixed with that ( EG `tinyRobot1` ) will be aggregated as part of this fleet.

* [Where it is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/launch/base.launch.xml#L32-L37)

### Fleet Adapters
Fleet Adapters are bridges between robot fleets and RMF. In this scenario, a robot state aggregator combines individual robots into a fleet. The Fleet Adapter then interfaces with the simulation fleet to control them.

We should **avoid** examining the exact mechanisms where the slotcar plugin communicates to the aggregator, and how that communicates with the Fleet Adapter. While the messages are exposed and open to view, it is not necessarily consistent across all robot fleets. 

We should consider all such messages as implementation-specific, and focus on how the Fleet Adapter APIs are used to bridge RMF and robot fleets.

* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/worlds/base/base.launch.xml#L33-L37)
* [Further Info](https://osrf.github.io/ros2multirobotbook/rmf-core.html#fleet-adapters)
* [Full Control Fleet Adapter API](https://osrf.github.io/ros2multirobotbook/integration_fleets.html#c-api)

### Fleet Adapter Parameters
We should note that the parameters given here are for the Fleet adapters developed for use with the simulation robots. While it is definitely possible to reuse these adapters for other systems, it is not a given that all fleet adapters exist in this form exact form.

It is very important that the fleet adapter parameters are accurate. The values here are projected onto the schedule, and so bad values here can disrupt proper scheduling. There are the following classes of parameters:
* Dynamics: These values describe how your robot moves physically
* RMF configurations: These values determine how RMF functions like deconflicting, task retrying should function, and what tasks are available in this fleet.
* Battery: RMF predicts battery consumption to plan recharging, so accurately modelling the battery is useful. Note that the battery is described as a value from 0 to 100, so if your system describes a value from 0 to 1, some normalization is needed.

* [Where It Is](https://github.com/open-rmf/rmf_gym/blob/main/rmf_gym_worlds/adapters/tinyRobot_adapter.launch.xml)
* [What is Vicinity?](https://osrf.github.io/ros2multirobotbook/rmf-core_faq.html?highlight=vicinity#what-distance-is-maintained-between-two-robots)
