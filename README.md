# ros-local-map-builder

## Overview
![](https://github.com/msc9533/ros-local-map-publisher/blob/master/doc/capture_rviz.png?raw=true)

This package is similar package with [costmap_2d](http://wiki.ros.org/costmap_2d). It provides extracting local map of 'base_link' frame from global map by using ros tf transform.  
But It also provide `odom` method. This method has no limitation with timestamp and more fast.

## Installation

Dependency  

```
$ sudo apt-get install ros-kinetic-map-server ros-kinetic-nav-msgs ros-kinetic-geometry-msgs
```

```
$ cd {your work space}
$ git clone https://github.com/msc9533/ros-local-map-publisher.git
$ catkin_make
```

## Usage

- It provide sample bag file to test package. To run this package with samplebag,

```
$ roslaunch local_map_builder test_with_bag.launch
```

- To run package with your own robot or own bag file,

```
$ roslaunch local_map_builder test.launch
```

