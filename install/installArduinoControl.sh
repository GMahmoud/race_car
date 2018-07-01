#!/bin/sh
sudo apt-get install arduino
sudo apt-get install ros-indigo-joy -y

# Copy Arduino code 
mkdir -p /home/maghob/sketchbook/libraries
cp -r Arduino\ Firmware/* '/home/maghob/sketchbook/libraries'
sudo apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial ros-indigo-angles -y
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries

