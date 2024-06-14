#!/bin/bash
echo "stop the service if running"
sudo systemctl stop dronecode-camera-manager.service
sudo systemctl disable dronecode-camera-manager.service	
sudo systemctl daemon-reload
echo "deploy on-start DCM"
cd ~/catkin_ws/src/camera-manager
make
echo "copying dronecode-camera-manager.service to /lib/systemd/system"
sudo cp /home/uvify/catkin_ws/src/camera-manager/dronecode-camera-manager.service /lib/systemd/system
echo "copying dcm to /usr/bin/dcm"
sudo cp /home/uvify/catkin_ws/src/camera-manager/dcm /usr/bin/dcm
echo "copying main.conf to /etc/dcm/main.conf"
sudo mkdir /etc/dcm/
sudo cp /home/uvify/catkin_ws/src/camera-manager/samples/config/main.conf /etc/dcm/main.conf

echo "enable and start the service"
sudo systemctl daemon-reload
sudo systemctl enable dronecode-camera-manager.service
sudo systemctl start dronecode-camera-manager.service	
echo "Done!"
