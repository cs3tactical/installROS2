#!/bin/bash
# 
# Copyright (c) 2021 Jetsonhacks 
# MIT License

# Roughly follows the 'Install ROS From Source' procedures from:
#   https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
# mostly from: 
#   Dockerfile.ros.foxy
#   https://github.com/dusty-nv/jetson-containers
# 

ROS_PKG=ros_base
ROS_DISTRO=humble
# Core ROS2 workspace - the "underlay"
ROS_BUILD_ROOT=/opt/ros/${ROS_DISTRO}-src
ROS_INSTALL_ROOT=/opt/ros/${ROS_DISTRO}


# Explicitly set UTF-8
echo 'export LANG=en_IL.UTF-8' >> ~/.bashrc
echo 'export LANGUAGE=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_CTYPE=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_NUMERIC=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_TIME=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_COLLATE=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_MONETARY=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_MESSAGES=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_PAPER=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_NAME=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_ADDRESS=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_TELEPHONE=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_MEASUREMENT=en_IL.UTF-8' >> ~/.bashrc
echo 'export LC_IDENTIFICATION=en_IL.UTF-8' >> ~/.bashrc

source ~/.bashrc
locale  # check for UTF-8

# Add the ROS 2 apt repository
# You will need to add the ROS 2 apt repository to your system.
# First ensure that the Ubuntu Universe repository is enabled.
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 apt repository
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
		curl \
		wget \ 
		gnupg \
		lsb-release
sudo rm -rf /var/lib/apt/lists/*

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list'

# wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc 
# sudo apt-key add ros.asc
# sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# install development packages
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
        python-is-python3 \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8-docstrings \
		python3-flake8 \
		python3-pip \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev \
		ros-dev-tools

sudo rm -rf /var/lib/apt/lists/*
  
# install some pip packages needed for testing
python3 -m pip install -U \
   argcomplete \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures

# compile yaml-cpp-0.8, which some ROS packages may use (but is not in the 20.04 apt repo)
git clone --branch 0.8.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.8 && \
    cd yaml-cpp-0.8 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    sudo cp libyaml-cpp.so.0.8.0 /usr/lib/aarch64-linux-gnu/ && \
    sudo ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.8.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.8


# https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
sudo mkdir -p ${ROS_BUILD_ROOT}/src && \
  cd ${ROS_BUILD_ROOT}
sudo sh -c "rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} launch_xml launch_yaml example_interfaces > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall"

# download unreleased packages     
sudo sh -c "git clone --branch ros2 https://github.com/Kukanani/vision_msgs ${ROS_BUILD_ROOT}/src/vision_msgs 
#     git clone --branch ${ROS_DISTRO} https://github.com/ros2/demos demos && \
#     cp -r demos/demo_nodes_cpp ${ROS_BUILD_ROOT}/src && \
#     cp -r demos/demo_nodes_py ${ROS_BUILD_ROOT}/src && \
#     rm -r -f demos"

# install dependencies using rosdep
sudo apt-get update
sudo apt upgrade
    cd ${ROS_BUILD_ROOT} 
sudo rosdep init  
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y --skip-keys "console_bridge fastrtps rfastcdr rti-connext-dds-6.0.1 urdfdom_headers" --os=ubuntu:jammy && \
    sudo rm -rf /var/lib/apt/lists/*

# build it!
sudo mkdir -p ${ROS_INSTALL_ROOT}
# sudo required to write build logs - try to build the heavy packages first
sudo colcon build --merge-install --install-base ${ROS_INSTALL_ROOT}  --packages-select rclpy rclcpp
# We do this twice to make sure everything gets built
# For some reason, this has been an issue
sudo colcon build --merge-install --install-base ${ROS_INSTALL_ROOT}

# Using " expands environment variable immediately
echo "source $ROS_INSTALL_ROOT/setup.bash" >> ~/.bashrc 
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
# echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
