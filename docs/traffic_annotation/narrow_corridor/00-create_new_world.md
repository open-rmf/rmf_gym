# Narrow Corridor - Create New World

In this section we will create a new world, using the `base` world as a point of reference. We will be creating a new world `narrow_corridor`.

First, you should have a look at the structure of launch files in [rmf_demos](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos/launch) and [this_repo](/launch) and understand what each component does. We will thus be making a copy of all the important launch files and modifying them for our `narrow_corridor` world.

You can use [this script](/rmf_gym_worlds/docs/generate_new_world.bash) to carry out all the following steps automatically:
```
NEW_SCENE=narrow_corridor bash generate_new_world.bash
```

## Explaining the Base World

## Boilerplate Steps to create a new world
We first copy and modify certain files. From the folder of `rmf_gym_worlds`:

We first define our new scene name:
```
export NEW_SCENE=narrow_corridor
```

We duplicate the main launch file and rename accordingly. This launch file contains all necessary processes for an RMF instance.
```
cp launch/base.launch.xml launch/$NEW_SCENE.launch.xml
# We then rename base to the new world name
sed -i "s/base/$NEW_SCENE/g" launch/$NEW_SCENE.launch.xml
```

This file contains the configuration of the rviz instance we will launch. We can customize it, such as the default floor for visualization.
```
mkdir -p launch/include/rviz/$NEW_SCENE
cp launch/include/rviz/base/base.rviz launch/include/rviz/$NEW_SCENE/$NEW_SCENE.rviz
```

These are the files that will modify to represent traffic lanes, and infrastructure like lifts and doors in our world, using the `traffic-editor`.
```
mkdir -p maps/$NEW_SCENE
cp -r maps/base/base.building.yaml maps/$NEW_SCENE/$NEW_SCENE.building.yaml
cp -r maps/base/base.png maps/$NEW_SCENE/$NEW_SCENE.png
sed -i "s/base/$NEW_SCENE/g" maps/$NEW_SCENE/$NEW_SCENE.building.yaml
```

These are test files we will use to check that our system works as expected.
```
mkdir -p launch/tests/$NEW_SCENE
cp launch/tests/base/base_basic_test.launch.xml launch/tests/$NEW_SCENE/$NEW_SCENE\_basic_test.launch.xml
cp launch/tests/base/base_random_loops_test.launch.xml launch/tests/$NEW_SCENE/$NEW_SCENE\_random_loops_test.launch.xml
```

For this world, we will demonstrate how to modify fleet adapters as well.  This is the new adapter we will use for our new fleet:
```
cp launch/include/adapters/deliveryRobot_adapter.launch.xml launch/include/adapters/corridorRobot_adapter.launch.xml
sed -i "s/deliveryRobot/corridorRobot/g" launch/include/adapters/corridorRobot_adapter.launch.xml
```

We should now have a World that is a duplicate of the `base` world, but with a different name.
