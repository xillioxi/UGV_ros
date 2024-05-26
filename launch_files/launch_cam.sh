#!/bin/bash

cd ros_ws/catkin_ws
source devel/setup.bash
roslaunch my_usb_cam usb_cam_test.launch 
