#!/bin/bash
cd ~
roscore &
timeout 10 wait $!
./launch_imu.sh &

timeout 10 wait $! 
./launch_cam.sh &

timeout 10 wait $!
./launch_orb.sh &
