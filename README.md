# Multi-Jackal Simulator using Gazebo ROS in Ubuntu 20/ROS Noetic

This package is a port of Nick Sullivan's multi-jackal simulator for Ubuntu 16. The ROS documentation can be found [here](http://wiki.ros.org/multi_jackal_tutorials).

# Installation Instructions

_This package is designed for Ubuntu 20/ROS Noetic. Make sure your system is running Ubuntu 20 and that ROS is installed: http://wiki.ros.org/noetic/Installation/Ubuntu_ (install the full desktop version: ros-noetic-desktop-full)

- Create a catkin workspace for your packages
```
mkdir -p ~/jackal_ws/src
cd ~/jackal_ws/
catkin_make
```
- Install and compile all the required base jackal noetic packages from clearpath:
```
cd src
git clone https://github.com/jackal/jackal.git
git clone https://github.com/jackal/jackal_simulator.git
git clone https://github.com/jackal/jackal_desktop.git
git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
cd ~/jackal_ws
rosdep install --from-paths . --ignore-src --rosdistro=noetic
catkin_make
```
- Install and compile this simulator in jackal_ws/src:
```
cd src
git clone https://github.com/rahulpeddi/multi-jackal-simulator
cd ~/jackal_ws
rosdep install --from-paths . --ignore-src
catkin_make
```
You are ready to run!

If you'd like to use move_base (optional), install the following ROS packages:
```
sudo apt-get install ros-noetic-geometry2
sudo apt-get install ros-noetic-navigation
```

# Overview
These packages make use of the robotic simulator Gazebo, along with the Jackal 
robot description. Multiple Jackals are spawned and are able to be moved 
independently. The difference between this package and the [Jackal package](https://github.com/jackal/jackal), 
is that multiple Jackals are able to be spawned without causing overlap. 
Significant amounts of code are from Clearpaths Jackal package, this just has 
minor changes.

If you only want to simulate one, then follow the 
[guide](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html). 
The problem is that it isn't scalable. They use the same transformation tree and 
some message names. You can see the problem yourself if you spawn two and have a 
look at the topics and TF tree.

# Files
## multi_jackal_tutorials
The starting point for simulating the robots. Contains launch, config, and world files.
Starts up a Gazebo session and launches robots using `multi_jackal_base`.
Example: `roslaunch multi_jackal_tutorials one_jackal.launch`.

## multi_jackal_base
Contains a single launch file that calls all other jackal components.

## multi_jackal_control
Launches the velocity controller plugin and robot controls.

## multi_jackal_description
Creates a plugin for publishing robot states and transformations. Loads a 
parameter that describes the robot for use in Gazebo.

## multi_jackal_nav
Creates the localization and move_base nodes.

# Running
Make sure the file `multi_jackal_description/scripts/env_run` is executable.

Example launch files can be found in `multi_jackal_tutorials/launch`. Gazebo can be viewed with `gzclient` in another terminal.

NOTE: rviz folder and config are included but Noetic's TF handling has been completely reworked, one robot can be visualized but multiple is not yet implemented
TODO: fix TF tree and rviz
