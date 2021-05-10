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
these test worlds can be used to support hardware testing: By having a standard dimensions, it's easy to reuse these scenarios for hardware-in-the-loop testing.

## Trying the base world
We can already try out the base world and a basic test case.

```
ros2 launch rmf_gym_worlds base.launch.xml
ros2 launch rmf_gym_worlds base_basic_test.launch.xml 
```

We should see the two robots looping in a cross-shape. When 10 Loops have been completed by each robot, the test process will terminate with a `0` exit status.

If the test case fails, the command will return 1. This happens when a timeout is reached. You can use this in our testing infrastructure to carry out contingency measures.

We can also run random loop tasks:
```
ros2 launch rmf_gym_worlds base_random_loops_test.launch.xml
```

## Tutorials
There are tutorials which you can work through to get a practical approach to RMF. You can see them [here](/docs).
