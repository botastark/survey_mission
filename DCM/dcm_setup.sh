#!/bin/bash
echo "installing necessary dependancies"
sudo apt-get update -y
sudo apt-get install git autoconf libtool python-pip -y
sudo apt-get install gstreamer-1.0 \
    libgstreamer-plugins-base1.0-dev \
    libgstrtspserver-1.0-dev -y
## Required python packages
echo "installing Required python packages"
sudo pip2 -q install -U future
# Avahi
echo "installing Required packages for Avahi"
sudo apt-get install libavahi-client-dev libavahi-core-dev libavahi-glib-dev -y

# To put camera-manager 
echo "Cloning Drone Camera Manager"
cd ~/catkin_ws/src/
git clone https://github.com/Dronecode/camera-manager.git
cd camera-manager
git submodule update --init --recursive
echo "replace for NNIDIA camera conf"
cp /home/uvify/catkin_ws/src/survey_mission/DCM/nvidia_camera.conf /home/uvify/catkin_ws/src/camera-manager/samples/config/main.conf
cp /home/uvify/catkin_ws/src/survey_mission/DCM/camera_def-jetsonkitcamera.xml /home/uvify/catkin_ws/src/camera-manager/samples/def/nvidia_camera.xml
cp /home/uvify/catkin_ws/src/survey_mission/DCM/ImageCaptureGst.cpp /home/uvify/catkin_ws/src/camera-manager/src/ImageCaptureGst.cpp
echo "check enable options"
./autogen.sh && ./configure --enable-mavlink --enable-avahi
echo "Make!"
make
echo "Done"

