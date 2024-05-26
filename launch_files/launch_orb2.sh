#!/bin/bash
cd ros_ws/catkin_ws
source devel/setup.bash
rosrun --prefix "gdp --args" orb_slam3_ros_wrapper  euroc_monoimu.launch
