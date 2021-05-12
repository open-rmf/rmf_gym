# Create New World

In this section we will create a new world, using the `base` world as a point of reference. We will be creating a new world `corridor`.
```
export NEW_WORLD=corridor 
```

## Boilerplate Steps to create a new world
We first make a copy of the `base` world. From `rmf_gym_worlds/worlds`:
```
cp -r base $NEW_WORLD
```

We first change the contents of all files:
```
cd $NEW_WORLD
grep -rl base . | xargs sed -i "s/base/$NEW_WORLD/g"
```

Then we rename the files themselves:
```
sudo apt install rename -y
cd $NEW_WORLD
find . -name '*base*' -exec rename base "$NEW_WORLD" {} +
```

Rebuild! From the workspace
```
rm -r install/rmf_gym_worlds
rm -r build/rmf_gym_worlds
colcon build --packages-select rmf_gym_worlds
```

We should now be able to run the new world:
```
ros2 launch rmf_gym_worlds corridor.launch.xml
``` 
