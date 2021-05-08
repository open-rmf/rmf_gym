#!/bin/bash

if [[ -z $NEW_SCENE ]]; then
   echo -e "Usage: NEW_SCENE=[new_world_name] bash generate_new_world.bash"
   exit 1
fi

script_path=$(dirname $(realpath "$0"))

# This launch file contains all necessary processes for an RMF instance.
cp $script_path/launch/base.launch.xml $script_path/launch/$NEW_SCENE.launch.xml
# We then rename base to the new world name
sed -i "s/base/$NEW_SCENE/g" $script_path/launch/$NEW_SCENE.launch.xml

# This file contains the configuration of the rviz instance we will launch. We can customize it, such as the default floor for visualization
mkdir -p $script_path/launch/include/rviz/$NEW_SCENE
cp $script_path/launch/include/rviz/base/base.rviz $script_path/launch/include/rviz/$NEW_SCENE/$NEW_SCENE.rviz

# These are the files that will modify to represent traffic lanes, and infrastructure like lifts and doors in our world.
mkdir -p $script_path/maps/$NEW_SCENE
cp -r $script_path/maps/base/base.building.yaml $script_path/maps/$NEW_SCENE/$NEW_SCENE.building.yaml
cp -r $script_path/maps/base/base.png $script_path/maps/$NEW_SCENE/$NEW_SCENE.png
sed -i "s/base/$NEW_SCENE/g" $script_path/maps/$NEW_SCENE/$NEW_SCENE.building.yaml

# This is a test file we will use to check a specific testing scenario
# We send two LoopRequests between two corresponding pairs for 10 loops each
mkdir -p $script_path/launch/tests/$NEW_SCENE
cp $script_path/launch/tests/base/base_basic_test.launch.xml $script_path/launch/tests/$NEW_SCENE/$NEW_SCENE\_basic_test.launch.xml

