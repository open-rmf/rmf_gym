# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure("2") do |config|
  config.vm.box = "bento/ubuntu-20.04"

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  config.ssh.forward_x11 = true
  config.vm.network "public_network"
  config.vm.define "rmf"
  config.vm.hostname = "rmf"
  config.vm.provider "virtualbox" do |v|
      v.cpus = 4
  end

  # Share an additional folder to the guest VM. The first argument is
  # the path on the host to the actual folder. The second argument is
  # the path on the guest to mount the folder. And the optional third
  # argument is a set of non-required options.
  # config.vm.synced_folder "../../workspaces", "/workspaces"

  # Enable provisioning with a shell script. Additional provisioners such as
  # Ansible, Chef, Docker, Puppet and Salt are also available. Please see the
  # documentation for more information about their specific syntax and use.
  config.vm.provision "shell", privileged: false, inline: <<-SHELL
     sudo apt update
     sudo apt install swapspace curl gnupg2 lsb-release python3-pip wget -y

     sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
     sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
     wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

     sudo apt update
     sudo apt install ros-foxy-desktop python3-argcomplete python3-rosdep -y 
     sudo apt install git cmake python3-vcstool curl qt5-default ignition-edifice -y
     python3 -m pip install flask-socketio
     sudo apt-get install python3-colcon* -y

     sudo rosdep init
     rosdep update
     mkdir -p ~/rmf_ws/src
     cd ~/rmf_ws
     wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
     vcs import src < rmf.repos
     rosdep install --from-paths src --ignore-src --rosdistro foxy -yr
     source /opt/ros/foxy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   SHELL
end
