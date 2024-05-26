# UGV_ros
This repo contains all of the ROS files and packages used for the final year project, for the autonomous vehicle

## Introduction
This project contains most of the source codes in my Final Year Project which was to build an autonomous vehicle which could traverse in rough terrain. Initailly the vehicle is based upon from the Donkey Car as a starting point https://www.donkeycar.com/ but modified with custom components in order to integrate with , the sensing, processing and controls are all performed on a Jetson Nano and is integrated with a camera, IMU, servo drivers and GPS for navigation and control.

I initially chosen this project because I wanted to learn what are the current limitations for small offroad autonomous vehicles and the components involved with creating such a project.

## Launch
These are the launch files I used primarily for starting the package and for easy debugging, for most of these packages it usually loads the setup.bash, inside the ros_ws and the execute the executables with the parameters, but typing in the executables everytime is very time consuming so I decided to put them into a launch file instead. 

## NMEA Files
These are the files used for GPS navigational unit, responsible for uses the ATGM336H navigation of the system, this is primarily used for the initital coordinates and the path finding used for the path finding algorithms. The broadcasts its location using the topics /fix, the setting can be modified inside of the launch file. 

## IMU Files
The IMU here used is based upon the JY901 IMU unit, it broadcasts the IMU data on the channel IMU_data, which contains the orientation, linear acceleration, which is used in detecting its location incombination with the SLAM data

## my_usb_cam
This is a launch file in order to launch broadcast the data from the USB camera to the topic camera/image_raw, which is used in the ORB_SLAM3 ROS wrapper package. Initially gscam is used for jetson's nano CSI cameras, but since both CSI camera ports broke, had to resort to using USB cameras.

## ORB_SLAM3_wrapper
This is the ORB_SLAM3 ROS wrapper which grabs the odometry, acceleration and keypoints data from the ORB_SLAM3 algorithm based on thien94's orb_slam3_wrapper which could be found at https://github.com/thien94/orb_slam3_ros_wrapper, which integrates with the ORB_SLAM3 repository found at https://github.com/UZ-SLAMLab/ORB_SLAM3. Initially the project sought to use Stereo Vision incombination with IMU in order to produce the best result, however it wasn't until it was figured out that the ORB_SLAM algorithm was too slow to run on the Jetson Nano and that code led to a lot of segmentation fault errors which was difficult to debug, that it was decided to use monocular vision without imu instead. Camera calibration was done using the CV2 library which to a lack of precision and accuracy, furthermore the camera was cheap off the shelf product which led to the algorithm inducing lots of noise which create a low quality pointcloud. In hindsight, visual odometry and 2D LiDAR may have been a better combination due to lower computational costs, furthermore since ORBSLAM primarily operates on keypoints, which made finding walls or objects with unicolor espicially difficult.

# Trajectory Planner
This folders contains most of the files which I used for the routing and control.

## D Star Planner/Global Path Planner
Initially, I thought to the D* Lite algorithm due to its success in other people's projects, but in my case I found it more useful to the Dynamic A* due to the assumption that this car would operate in a remote area with no dynamic objects people etc, this meant that the replanner actually performs almost in linear time to the size of the area getting blocked off. The planner initially the Dynamic A* planner senses for if the goal_node has been defined, which is a longitude and a latitude coordinate broadcasted on the goal_node topic, if the goal node is received, then it begins initializing the map, based on the manhattan distances for the entire grid. Currently, this planner doesn't support loading an prestored maps, therefore the weights of the nodes are always the same during the initialization. 
**Problem**
The initialization phase is that the algorithm wasn't implemented using Quadtrees, which meant that the running time is O(n^2) where n is the x or y dimension, if given more time I believe the implementation of Quadtrees would be significantly increase the running speed and definition of the algorithm during initialization.

## createPointcloud / pointCloudProcessing
This is the pointCloud processing node which uses PointCloudLibrary for segmentation and classification, which takes PointCloud2 points from the topic point_cloud, then the algorithm first checks whether a point is already located inside of the map or not then it is added, then the ground is extracted from the pointcloud using RANSAC method, the pointcloud is then voxelized based on the pointcloud density and relative height from the ground.

**Problem**
The problem with this implementation is that since it is not implemented using Octrees yet, the iteration of each algorithm meant it would operate on the entire dataset of points, therefore the speed of the algorithm would scale linearly with n the number of points, hence after n>100,000 it would become too slow to operate in realtime and hasn't implemented a system to drop frames yet, so therefore it would clog up the system and deem it unusable. I believe in the future using a kNN search using octrees will make this much more performant.

# Conclusions and findings
A lot of algorithms involved really toke a lot of computational power, furthermore I found that the current limtations of running computationally heavy algorithms on edge devices are one of the main problems in preventing small autonomous vehicles, for example ORB_SLAM3 and real time PointCloud processing all ran longer than ideal time, furthermore development time is significantly increased when trying to build software on the edge device instead of performing cross compilation. In this project due to a time constraint, Whilst the car was operational and can accelerate and turn and each component was working individually, I was unable to finish all parts of the control algorithms due to time constraints from too large of a project scope and many build issues with the ORB SLAM3 codebase. Furthermore, many of the algorithms needed optimization using data structures, such as path planning using a 2D grid instead of Quadtree and pointcloud segmentation not using Octrees, meant that there were servere performance regarding space and time complexity which will require a lot of time to test and fix.
