# ros-local-map-builder

## Overview
![](https://github.com/msc9533/ros-local-map-publisher/blob/master/doc/capture_rviz.png?raw=true)



[costmap_2d](http://wiki.ros.org/costmap_2d)와 비슷한 기능을 하는 패키지 입니다.  
하지만 `costmap_2d` 패키지와 달리 `odom`을 이용한 방식을 제공합니다. 이 방식은 `tf`와 달리 시간에 대한 제약을 받지 않습니다. 
