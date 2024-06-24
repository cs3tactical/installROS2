# installROS2
Scripts to install ROS2 (humble) on the NVIDIA Jetson Development Kits

This is a simple script to install ROS2 on the NVIDIA Jetson Development Kits.

In order to run the script:

<blockquote>$ ./installROS2</blockquote>

The script roughly follows the 'Install ROS From Source' procedures from:

<blockquote>https://index.ros.org/doc/ros2/Installation/Humble/Linux-Development-Setup/</blockquote>

Much of the code is taken from the dusty-nv Github repository jetson-containers. The dusty-nv jetson-containers should be used to create a Docker container for the ROS2 on the Jetson. For more information:

<blockquote>
Dockerfile.ros.humble
https://github.com/dusty-nv/jetson-containers
</blockquote> 

<h3>Notes</h3>
Currently the NVIDIA Jetsons run Ubuntu 20.04. ROS2 humble requires Ubuntu 20.04. However, by providing some of the 20.04 equivalents, it is possible to run ROS2 humble on the Jetsons. This script provides workarounds. Note that this is not exhaustive, you may run into situations where there are gaps, and certain packages may have issues.

<br><p>The file ~/.bashrc is modified using this script; You will want to make sure that the modifications match your expectations. For example, you may have a preference for having the install/setup.bash in another place (i.e. /root/.bashrc).
 

<h3>Release Notes</h3>

<b>June, 2024</b>
* Changed to build ROS2 Humble instead of Foxy
* Tested on JetPack 5.12 L4T 35.4.1
* Tested on Jetson Orin Nano 8GB (with NVME)

<b>January, 2021</b>
* Tested on JetPack 4.5, L4T 32.5
* Tested on Jetson Xavier NX
