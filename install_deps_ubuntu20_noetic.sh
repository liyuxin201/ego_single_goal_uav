#!/usr/bin/env bash
set -euo pipefail
sudo apt update
sudo apt install -y   build-essential cmake git python3-catkin-tools python3-rosdep python3-vcstool   ros-noetic-desktop-full   ros-noetic-mavros ros-noetic-mavros-extras   ros-noetic-vision-msgs ros-noetic-pcl-ros ros-noetic-octomap-ros   ros-noetic-tf ros-noetic-tf2-ros ros-noetic-rviz   libeigen3-dev libnlopt-dev libgoogle-glog-dev libgflags-dev   libopencv-dev libpcl-dev libarmadillo-dev libsuitesparse-dev
