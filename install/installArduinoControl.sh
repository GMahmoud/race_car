#!/bin/sh
sudo apt-get install arduino
sudo apt-get install ros-indigo-joy -y

# Copy Arduino code 
mkdir -p $HOME/sketchbook/libraries
cp -r arduino_firmware/* '$HOME/sketchbook/libraries'
sudo apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial ros-indigo-angles -y
cd $HOME/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries

