#!/bin/bash

cd ~/ros1Gps_ws/
source devel/setup.bash
roslaunch nmea_navsat_driver nmea_serial_driver.launch

