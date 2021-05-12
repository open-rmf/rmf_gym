# rmf_gym_worlds
This repository contains simulation test worlds to help check that common multi-robot scenarios play out correctly.

# Overview
The repository structure is:
```
rmf_gym_worlds
  adapters                                  # Contain common fleet adapter launch files and configurations
  rviz                                      # Contain common rviz configurations
  worlds                                    # All simulation worlds
    base                                    # An individual world 
      docs                                  # Documentation and resources on this world
      maps                                  # building.yaml files and other resources
      tests                                 # Tests
        base_sim_setup.launch.xml           # Launch file that spawns simulation robots          
        test_base_...launch.xml             # Individual test task cases
      base.launch.xml                       # The main launch file for this world
    ...                                     # Other test worlds
  common.launch.xml                         # Common launch nodes
  simulation.launch.xml                     # Simulation launch nodes
```

# The base world
The `Base` world is a template world with standard dimensions `10m X 20m` meant to standardize the dimensions of testing scenarios. The standard sizes should hopefully make it more consistent to test out hardware-in-the-loop scenarios.

## Trying the base world
We can already try out the base world and a basic test case.

```
# Launch simulation world and RMF backend
ros2 launch rmf_gym_worlds base.launch.xml use_sim_time:=true                         # Launch simulation and RMF backend headless will prevent gazebo GUI from opening

# Test 
ros2 launch rmf_gym_worlds base_sim_setup.launch.xml                                  # Spawn Robots: Modify this file to change initial startup locations
ros2 launch rmf_gym_worlds test_base_simple_loop.launch.xml                       # Run any number of tests

# Teardown
Ctrl-C                                                                                # Stop RMF Backend
```

We should see the two robots looping in a cross-shape. When 10 Loops have been completed by each robot, the test process will terminate with a `0` exit status.

If the test case fails, the command will return 1. This happens when a timeout is reached. You can use this in our testing infrastructure to carry out contingency measures.

For more details information, have a look at [Examining the Base World](/docs/rmf_structure/02-examining-the-base-world).

## Trying out other worlds
The same commands above should translate to testing other worlds as well, by replacing `base` with your target world.

## Tutorials
There are tutorials which you can work through to get a practical approach to RMF. You can see them [here](/docs).
