#!/bin/bash

### Change Swap Memory

# Turn swap off
# This moves stuff in swap to the main memory and might take several minutes
sudo swapoff -a

# Create a 8GB Swap
sudo dd if=/dev/zero of=/swapfile bs=1G count=8
sudo chmod 0600 /swapfile

sudo mkswap /swapfile  # Set up a Linux swap area
sudo swapon /swapfile  # Turn the swap on

### Install CMake New Version
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates gnupg software-properties-common wget
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt-get update
sudo apt-get install cmake


### Install Pangolin 
cd ~ 
git clone https://github.com/stevenlovegrove/Pangolin.git
sudo apt install libglew-dev
cd Pangolin && mkdir build && cd build
cmake ..
make -j 4
sudo make install

### Install Googlelog
cd ~
git clone https://github.com/google/glog
cd glog
cmake -H. -Bbuild -G "Unix Makefiles"
cmake --build build
cmake --build build --target test
cd build
sudo make install


### Install OpenCV
cd ~
git clone https://github.com/opencv/opencv
git -C opencv checkout 4.5.1
cd opencv 
mkdir build
cmake ..
cmake -j4


