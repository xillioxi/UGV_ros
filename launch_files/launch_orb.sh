#!/bin/bash
cd ros_ws/catkin_ws
source devel/setup.bash
roslaunch orb_slam3_ros_wrapper  euroc_mono.launch
