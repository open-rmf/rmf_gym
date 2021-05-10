# Create  Simple Lift World

In this section we will create a new world, using the `base` world as a point of reference. We will be creating a new world `simple_lift`.

You can use [this script](/rmf_gym_worlds/docs/generate_new_world.bash) to carry out all the following steps automatically:
```
NEW_SCENE=simple_lift bash generate_new_world.bash
```

## Boilerplate Steps to create a new world
We first copy and modify certain files. From the folder of `rmf_gym_worlds`:

We duplicate the main launch file and rename accordingly. This launch file contains all necessary processes for an RMF instance.
```
cp launch/base.launch.xml launch/simple_lift.launch.xml
# We then rename base to the new world name
sed -i "s/base/simple_lift/g" launch/simple_lift.launch.xml
```

This file contains the configuration of the rviz instance we will launch. We can customize it, such as the default floor for visualization.
```
mkdir -p launch/include/rviz/simple_lift
cp launch/include/rviz/base/base.rviz launch/include/rviz/simple_lift/simple_lift.rviz
```

These are the files that will modify to represent traffic lanes, and infrastructure like lifts and doors in our world, using the `traffic-editor`.
```
mkdir -p maps/simple_lift
cp maps/base/base.building.yaml maps/simple_lift/simple_lift.building.yaml
cp maps/base/base.png maps/simple_lift/simple_lift.png
sed -i "s/base/simple_lift/g" maps/simple_lift/simple_lift.building.yaml
```

These are test files we will use to check that our system works as expected.
```
mkdir -p launch/tests/simple_lift
cp launch/tests/base/base_basic_test.launch.xml launch/tests/simple_lift/simple_lift_basic_test.launch.xml
cp launch/tests/base/base_random_loops_test.launch.xml launch/tests/simple_lift/simple_lift_random_loops_test.launch.xml
```

