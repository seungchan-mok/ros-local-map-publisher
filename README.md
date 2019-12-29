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


## Configuration

```
source_frame: "/map"
child_frame: "/base_link"
map_topic: "/map"
method: "odom" # tf or odom only
odom_topic: "/odom"
cost_map: true
pointcloud_topic: "/pcl"
global_map_static: true
map_size: [5.0, 5.0] #x,y (meter)
tolerance: 0.3 #tf 방식을 이용할때 설정하는 시간오차입니다.
cost: 0.2
```

- `source_frame : "/map"`global map이 publish되고 있는 프레임입니다. 보통 `/map`으로 되어 있습니다.
- `child_frame : "/base_link"`local_map 이 publish되어야 할 프레임입니다. 보통`/base_link`를 씁니다.
- `map_topic : "/map"`map_server를 이용한 static_map이 아닌 map이 publish되고 있는 경우를 위한 topic이름입니다.  
- `method : "tf"` `tf` 방법, `odom`방법 두가지중 하나를 사용합니다.
- `cost_map : true`laser scan을 이용한 cost_map의 기능을 on/off합니다.
- `pointcloud_topic : /scan`cost map을 표시할 point cloud의 topic이름입니다. 3D pointcloud일 경우 z축은 무시됩니다.
- `global_map_static: true` map_server 를 이용한 static map을 사용할 경우 true, publish되는 topic일 경우 false를 입력 합니다.
- `map_size : [5.0 5.0]` map size를 입력합니다. 만들어지는 map은 로봇의 원점 기준으로 +-meter만큼 생성됩니다.
- `tolerance: 0.3` tf 방법을 사용할 경우 허용되는 시간 오차입니다.
- `cost: 0.2` : laser scan 등을 

### Configuration of `tf` Method

tf 방법을 사용할때는 다음과 같이 설정되어 있는것을 확인하여야 합니다.


![](https://github.com/msc9533/ros-local-map-publisher/blob/master/doc/tf_tree.png?raw=true)

[tf_tutorials](http://wiki.ros.org/tf/Tutorials)

### Configuration of `odom` Method

`odom` 방법을 사용할때는 publish되고 있는 odometry의 frame이 global map의 frame과 같아야 합니다.

- 내용추가예정.

### Configuration of cost map

추가예정.

### Configuration of `combined` Method

추가예정.