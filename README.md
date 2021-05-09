# rmf_gym

Welcome! `rmf_gym` aims to combine a few different goals for the [Robotics Middleware Framework](https://github.com/open-rmf/rmf_demos) ( RMF ):
* Practical Tutorials
* Testing infrastructure 
* Documentation

## rmf_gym_worlds
The package [`rmf_gym_worlds`](/rmf_gym_worlds) contains many RMF worlds that are optimized for focused simulation testing. There are also tutorials that teach you how to craft your own testing world, which doubly act as tutorials on how to use RMF. 

It is easy to tweak the gym worlds to include your hardware for testing, by physically running your robot alongside simulations in a physical `10m x 20m` space.

## rmf_gym_tools
The package [`rmf_gym_tools`](/rmf_gym_tools) contain scripts and automation that help with testing RMF. These can be run standalone or put into [test launch files](/rmf_gym_worlds/launch/tests/base). You can send pre-determined tasks to replicate tricky scenarios, or infinitely send random tasks to see how long the system lasts.

## Vagrant Machine Image
A `Vagrantfile` is provided to set up a VM with `rmf_gym`. This helps make to set up a consistent environment quickly, regardless of your current setup. [Install Vagrant](https://www.vagrantup.com/docs/installation) and [install VirtualBox](https://www.virtualbox.org/wiki/Linux_Downloads), then run:
```
# For virtualbox, you might need to install python
sudo apt install python ( python-is-python2 )

vagrant up                # Provision VM
vagrant ssh-config        # For ssh information
vagrant ssh               # Get a terminal

# You can now run RMF and tools!
source /opt/ros/foxy/setup.bash
source $HOME/rmf_ws/install/setup.bash
ros2 launch rmf_demos office.launch.xml
```

## Examples

### base
![base.gif](/docs/base.gif)

### narrow_corridor
![narrow_corridor.gif](/docs/narrow_corridor.gif)

### simple_lift
![simple_lift.gif](/docs/simple_lift.gif)

More to come!
