packer {
  required_plugins {
    docker = {
      version = ">= 0.0.7"
      source = "github.com/hashicorp/docker"
    }
    virtualbox = {
      version = ">= 0.0.1"
      source = "github.com/hashicorp/virtualbox"
    }
  }
}

variable "ros2_version" {
  type    = string
  default = "foxy"
}

variable "build_command" {
  type    = string
  default = "MAKEFLAGS=\"-j1 -l1\" colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release"
}

source "docker" "rmf" {
  image  = "ubuntu:focal"
  commit = true
}

source "vagrant" "rmf" {
  source_path  = "bento/ubuntu-20.04"
  provider = "virtualbox"
  communicator = "ssh"
  add_force = true
}

build {
  sources = [
    "source.docker.rmf",
    "source.vagrant.rmf",
  ]
  
  provisioner "shell" {
    inline = [
      "apt update && apt install sudo -y"
    ]
    only = ["docker.rmf"]
  }

  # ROS2
  provisioner "shell" {
    inline = [
      "apt update && apt install curl gnupg2 lsb-release wget -y",
      "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg",
      "echo \"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main\" | tee /etc/apt/sources.list.d/ros2.list > /dev/null",
      "apt update",
      "DEBIAN_FRONTEND=noninteractive apt install ros-${var.ros2_version}-desktop -y", 
      "apt install python3-rosdep -y",
      "rosdep init",
      "rosdep update"
    ]
    execute_command = "echo 'vagrant' | {{ .Vars }} sudo -E -S sh '{{ .Path }}'"
  }

  # RMF
  provisioner "shell" {
    inline = [
      "sh -c 'echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main\" > /etc/apt/sources.list.d/gazebo-stable.list'",
      "wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -",
      "apt update",
      "apt install git cmake python3-vcstool curl qt5-default ignition-edifice python3-pip -y",
      "yes | python3 -m pip install flask-socketio",
      "apt-get install python3-colcon* -y",
      "mkdir -p ~/rmf_ws/src",
      "cd ~/rmf_ws",
      "wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos",
      "vcs import src < rmf.repos",
      "rosdep install --from-paths src --ignore-src --rosdistro foxy -yr",
      ". /opt/ros/${var.ros2_version}/setup.sh && ${var.build_command}"
    ]
    execute_command = "echo 'vagrant' | {{ .Vars }} sudo -E -S sh '{{ .Path }}'"
  }

    post-processor "docker-tag" {
    repository = "open-rmf"
    tags       = ["ubuntu-focal", "rmf"]
    only       = ["docker.rmf"]
  }
}
