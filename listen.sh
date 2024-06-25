#!/bin/bash
sudo systemctl stop listen.service 
sudo systemctl stop listen_tol.service 
sudo systemctl disable listen.service 
sudo systemctl disable listen_tol.service

sudo cp ~/catkin_ws/src/survey_mission/listen.service /etc/systemd/system/listen.service
sudo cp ~/catkin_ws/src/survey_mission/listen_tol.service /etc/systemd/system/listen_tol.service

systemctl daemon-reload
sudo systemctl enable listen.service 
sudo systemctl enable listen_tol.service 
sudo systemctl start listen.service 
sudo systemctl start listen_tol.service
