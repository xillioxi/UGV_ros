# UGV_ros
This repo contains all of the ROS files and packages used for the ROS ecosystem, for the autonomous vehicle

## Introduction
This proejct contains most of the source codes I used in my Final Year Project which was to build an autonomous vehicle which could traverse in rough terrain. Initailly the project is based upon from the Donkey Car Project as a starting point https://www.donkeycar.com/, this project runs on a Jetson Nano and is integrated with a camera, IMU, servo drivers and GPS for navigation and control of the vehicle.


## Launch
These are the launch files I used primarily for starting the package and for easy debugging, for most of these packages it usually loads the setup.bash, inside the ros_ws and the execute the executables with the parameters, but typing in the executables everytime is very time consuming so I decided to put them into a launch file instead

## NMEA Files
These are the files used for GPS navigational unit, responsible for uses the ATGM336H navigation of the system, this is primarily used for the initital coordinates and the path finding used for the path finding algorithms. The broadcasts its location using the topics /fix, the setting can be modified inside of the launch file. 

## IMU Files
The IMU here used is based upon the JY901 IMU unit, it broadcasts the IMU data on the channel IMU_data, which contains the orientation, linear acceleration, which is used in detecting its location incombination with the SLAM data

## my_usb_cam
This is a launch file in order to launch broadcast the data from the USB camera to the topic camera/image_raw, which is used in the ORB_SLAM3 ROS wrapper package. Initially gscam is used for jetson's nano CSI cameras, but since both CSI camera ports broke, had to resort to using USB cameras.

## ORB_SLAM3_wrapper
This is the ORB_SLAM3 ROS wrapper which grabs the odometry, acceleration and keypoints data from the ORB_SLAM3 algorithm based on thien94's orb_slam3_wrapper which could be found at https://github.com/thien94/orb_slam3_ros_wrapper, which integrates with the ORB_SLAM3 repository found at https://github.com/UZ-SLAMLab/ORB_SLAM3.

# Trajectory Planner
This folders contains most of the files which I used for the routing and control.

## D Star Planner
This is planner whichi 
