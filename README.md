# Package Name

## Overview

This is a package about operator manipulation, including data reception, processing and transmission when the computer
or remote controller controls the robot.

**Keywords:** auxiliary, ROS

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: QiayuanLiao<br />
Affiliation: [Dynamicx]()<br />
Maintainer: QiayuanLiao, liaoqiayuan@gmail.com**

The rm_hw package has been tested under [ROS] Melodic and Noetic on respectively 18.04 and 20.04. This is research code,
expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)

![Example image](doc/example.jpg)

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...

Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- roscpp
- sensor_msgs
- roslint
- rm_msgs
- rm_common
- tf2_geometry_msgs
- control_msgs
- controller_manager_msgs

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone http://frp.acmetech.top:8080/dynamicx/rm_manual.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

### Unit Tests

Run the unit tests with

	catkin_make text_power_limit.launch

### Static code analysis

Run the static code analysis with

	catkin_make roslint_ros_package_template

## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch rm_manual load.launch

Or better, use `rm_config`:

```
roslaunch rm_config manual.launch
```

## Config files

Config file rm_manual/config

* **engineer.yaml** Shortly explain the content of this config file
