#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
#start roscore in background
roscore &
ROSCORE_PID=$!
sleep 5

roslaunch survey_mission survey_camera.launch --wait

# Start the receive_file.py script in the background

wait $ROSCORE_PID



