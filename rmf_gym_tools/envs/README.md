# envs
This section contains various examples on how to get started with RMF.

## Bare Metal
The traditional approach to installing RMF is directly on your devices. This option provides the most flexibility in developing RMF. Follow the instructions [here](https://github.com/open-rmf/rmf).

## Machine Images
We use [packer](https://www.packer.io/) to build machine images. These machine images are alternatives to the bare-metal method that could fit different use cases. We currently have `Docker` and `Vagrant` machine images, for Containers and Virtual Machines.

### Setup
To use the following images, you will need to do some extra setup. The tradeoff is a more consistent execution environment.

```
# For Docker: https://docs.docker.com/engine/install://docs.docker.com/engine/install/
# For Vagrant: https://www.vagrantup.com/docs/installations://www.vagrantup.com/docs/installation
```

### Build Locally
You can build the images locally using the `rmf.pkr.hcl` file:
```
# Install Packer based on your manager: https://www.packer.io/downloads

# Build everything
packer build rmf.pkr.hcl        
# Build only Docker
packer build -only=docker.rmf rmf.pkr.hcl
# Build only Vagrant
packer build -only=vagrant.rmf rmf.pkr.hcl
```

Builds might take a long time, as the `pkr.hcl` file is configured to build with minimal resources. If you have more RAM, you can replace the build command variable:
```
packer build -var='"colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"' rmf.pkr.hcl
```

### Use Pre-Built Machine Images
You can also choose to pull the pre-built images we upload to `Docker Hub` and `Vagrant Cloud`:
```
# Pull Docker image
# Pull Vagrant Box
TODO
```

### Run Machine Images
Now that you have either built or pulled the Machine Images, you can run them
```
# For Docker
docker-compose up 

# For Vagrant
vagrant up
vagrant ssh
```

You should now see an RMF-able via the given provider. You should be able to run the following there:
```
source /opt/ros/foxy/setup.bash
source $HOME/rmf_ws/install/setup.bash
ros2 run rmf_demos office.launch.xml headless:=1
```
