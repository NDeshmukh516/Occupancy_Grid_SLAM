# Occupancy Grid SLAM using Hector 
by Anthony Ambrose and Nikhil Deshmukh

1. Setup ROS Kinetic on Ubuntu 16.04. Follow alternate instructions to install updated version of Gazebo, at http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0

2. Download the hector_quadrotor ROS package for ROS Kinetic with the instructions found at https://darienmt.com/autonomous-flight/2018/10/20/flying-ros-and-hector.html

To install, follow:

```wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/kinetic-devel/tutorials.rosinstall

sudo apt-get install ros-kinetic-geographic-info

sudo apt-get install ros-kinetic-ros-control

sudo apt-get install ros-kinetic-gazebo-ros-control

sudo apt-get install ros-kinetic-joy

sudo apt-get install ros-kinetic-move-base

sudo apt-get install ros-kinetic-octomap_server

sudo apt-get install ros-kinetic-octomap-ros

sudo apt-get install ros-kinetic-octomap

sudo apt-get install ros-kinetic-teb-local-planner

sudo apt-get install ros-kinetic-pointcloud-to-laserscan

sudo apt-get install ros-kinetic-ros-numpy

catkin_make

source devel/setup.bash ```

3. The scripts in this project requires numpy, rospy, math, tf2_ros, all of which can be installed using the pip/pip3 command in the terminal. 



4. To launch the X-Y mapping script:

roslaunch drone_slam SlamExploreIndoors.launch
rosrun drone_slam explorer.py



5. To launch the Z-axis mapping script, rename molten_coreELEVATE to molten_core, and rename molten_core to a different name.

roslaunch drone_slam SlamElevation.launch
rosrun drone_slam explorer.py

The only difference between the launch files is the kind of map it opens up, with the elevation launch file being a vertical tower to demonstrate Z-axis mapping.
