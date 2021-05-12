# rmf_gym

Welcome! `rmf_gym` aims to provide the following for the [Robotics Middleware Framework](https://github.com/open-rmf/rmf_demos) ( RMF ):
* Practical Tutorials
* Testing infrastructure 
* Documentation

## Tutorials
This gym means to offer a more hands-on approach to learning RMF, we will make reference to the [ROS2 Multi Robot Book](https://osrf.github.io/ros2multirobotbook/) for extensive explanations.

The practical tutorials can be found [here](/docs).

## rmf_gym_worlds
The package [`rmf_gym_worlds`](/rmf_gym_worlds) contains many RMF worlds that are optimized for focused simulation testing. There are also tutorials that teach you how to craft your own testing world. They are also standard in size to help hardware in the loop testing.

It is easy to tweak the gym worlds to include your hardware for testing, by physically running your robot alongside other simulated robots / infrastructure. 

## rmf_gym_tools
The package [`rmf_gym_tools`](/rmf_gym_tools) contain scripts and automation that help with testing RMF. These can be run standalone or put into [test launch files](/rmf_gym_worlds/worlds/base/tests). You can send pre-determined tasks to replicate tricky scenarios, or infinitely send random tasks to see how long the system lasts.

## Vagrant Machine Image
A `Vagrantfile` is provided to create a Virtual Machine with `rmf_gym` all set up. This could be a quick way to try out `rmf_gym`, although there is a pretty significant performance penalty to it.

Note that some devices might encounter a GLX context error when running graphical interfaces like `rviz` and `gazebo` but others might not. I'm not entirely sure what's wrong yet, but it probably has to do with graphics drivers you are running.

[Install Vagrant](https://www.vagrantup.com/docs/installation) and [install VirtualBox](https://www.virtualbox.org/wiki/Linux_Downloads), then run:
```
# For virtualbox, you might need to install python
sudo apt install python ( python-is-python2 )
# You should also modify the Vagrantfile according to the amount of RAM you can provide to the VM, at least 8GB if you build from source.

vagrant up                # Provision VM
vagrant ssh-config        # For ssh information
vagrant ssh               # Get a terminal

# You can now run RMF and tools on the provisioned VM
source /opt/ros/foxy/setup.bash
source $HOME/rmf_ws/install/setup.bash
ros2 launch rmf_demos office.launch.xml

# If the GUI is giving errors or is too slow
ros2 launch rmf_demos office.launch.xml headless:=1
```

Alternatively, you can also download a pre-built box from [Vagrant Cloud](https://app.vagrantup.com/cnboonhan/boxes/rmf):
```
VAGRANT_VAGRANTFILE=Vagrantfile.cloud vagrant up
VAGRANT_VAGRANTFILE=Vagrantfile.cloud vagrant ssh

source /opt/ros/foxy/setup.bash
source $HOME/rmf_ws/install/setup.bash
ros2 launch rmf_demos office.launch.xml

# If the GUI is giving errors or is too slow
ros2 launch rmf_demos office.launch.xml headless:=1
```

## Examples

### base
![base.gif](/docs/gifs/base.gif)

### simple_door
![simple_door.gif](/docs/gifs/simple_door.gif)

### simple_lift
![simple_lift.gif](/docs/gifs/simple_lift.gif)

More to come!
