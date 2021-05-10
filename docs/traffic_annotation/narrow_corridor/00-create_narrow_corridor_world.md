# Create Narrow Corridor World

In this section we will create a new world, using the `base` world as a point of reference. We will be creating a new world `narrow_corridor`.

You can use [this script](/rmf_gym_worlds/docs/generate_new_world.bash) to carry out all the following steps automatically:
```
NEW_SCENE=narrow_corridor bash generate_new_world.bash
```

For this world, we go the extra mile to modify the fleet name from `deliveryRobot` to `corridorRobot`, so some additional steps are needed.

## Boilerplate Steps to create a new world
We first copy and modify certain files. From the folder of `rmf_gym_worlds`:

We first define our new scene name:
```
export NEW_SCENE=narrow_corridor
```

We duplicate the main launch file and rename accordingly. This launch file contains all necessary processes for an RMF instance.
```
cp launch/base.launch.xml launch/narrow_corridor.launch.xml
# We then rename base to the new world name
sed -i "s/base/narrow_corridor/g" launch/narrow_corridor.launch.xml
```

This file contains the configuration of the rviz instance we will launch. We can customize it, such as the default floor for visualization.
```
mkdir -p launch/include/rviz/narrow_corridor
cp launch/include/rviz/base/base.rviz launch/include/rviz/narrow_corridor/narrow_corridor.rviz
```

These are the files that will modify to represent traffic lanes, and infrastructure like lifts and doors in our world, using the `traffic-editor`.
```
mkdir -p maps/narrow_corridor
cp -r maps/base/base.building.yaml maps/narrow_corridor/narrow_corridor.building.yaml
cp -r maps/base/base.png maps/narrow_corridor/narrow_corridor.png
sed -i "s/base/narrow_corridor/g" maps/narrow_corridor/narrow_corridor.building.yaml
```

These are test files we will use to check that our system works as expected.
```
mkdir -p launch/tests/narrow_corridor
cp launch/tests/base/base_basic_test.launch.xml launch/tests/narrow_corridor/narrow_corridor\_basic_test.launch.xml
cp launch/tests/base/base_random_loops_test.launch.xml launch/tests/narrow_corridor/narrow_corridor\_random_loops_test.launch.xml
```

For this world, we will demonstrate how to modify fleet adapters as well.  This is the new adapter we will use for our new fleet:
```
cp launch/include/adapters/deliveryRobot_adapter.launch.xml launch/include/adapters/corridorRobot_adapter.launch.xml
sed -i "s/deliveryRobot/corridorRobot/g" launch/include/adapters/corridorRobot_adapter.launch.xml
```

We should now have a World that is a duplicate of the `base` world, but with a different name, as well as a modified fleet adapter.

