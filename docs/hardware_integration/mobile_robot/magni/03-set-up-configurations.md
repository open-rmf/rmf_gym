# Set Up Configurations

We will to make sure that network configuration has been set up properly. We will use the `fake_server` and `fake_client` to do a sanity check on our network configuration.

On your workstation:

```
# Source your ROS2 and free fleet workspaces
ros2 launch ff_examples_ros2 fake_server.launch.xml
```

On your Magni:
```
# Source your ROS2 and free fleet workspaces
roslaunch ff_examples_ros1 fake_client.launch.xml
```

You should see some updates on the `fake_server` logs:

```
found new robot: fake_robot
```

# Use Real launch files
If the test works, we can try launching the real deal:

On your workstation:

```
# Source your ROS2 and free fleet workspaces
ros2 launch [your-free-fleet-server-package] free_fleet_server.launch.xml
```

On your Magni:
```
# Source your ROS2 and free fleet workspaces
roslaunch [your-free-fleet-client-package] free_fleet_client.xml
```

We should hopefully see an update on the `free_fleet_server` logs:
```
found new robot: ubiquityrobot
```

And we should be able to echo `fleet_states` as output by the free fleet server:
```
# Source your RMF workspace
ros2 topic echo /fleet_states
```

Launch a visualization:
```
ros2 launch rmf_gym_worlds base.launch.xml
```
And somewhere on the map, you should be able to see a purple dot. If so, congratulations, the plumbing is done!

What is left is to configure the coordinate transforms of free fleet correctly, so that the purple blob is at the correct place.

## Common issues
Sometimes this connection doesn't work, and it might be challenging to debug. Here are some possible debugging steps:

### You must be able to ping each other
```
ping ubiquityrobot.local        # From workstation, to robot
ping workstation.local          # From robot, to your workstation 
```

### You must have suitable iptables to not drop packets. If you can, try
```
# (as sudo )
iptables -F
iptables -F -t nat
```

### You must have suitable firewall to not block packets. If you can, try
```
# (as sudo)
ufw disable       # Or whatever firewall you use
```

### You must preferably not multi-home your workstation and robot, meaning they should only connect to one network.
### If you need to multi-home, you should ideally use a configuration file to restrict CycloneDDS to only your RMF network
```
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml          # Place this in .bashrc

# Find out your RMF network interface
ip a

# Add the following lines to /etc/cyclonedds.xml, replacing "wlan0" with your RMF network interface
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
            <AllowMulticast>default</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
            <FragmentSize>4000B</FragmentSize>
        </General>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
        <Tracing>
            <Verbosity>config</Verbosity>
            <OutputFile>stdout</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```
