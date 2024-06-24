#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
#start roscore in background
roscore &
ROSCORE_PID=$!
sleep 5

roslaunch survey_mission survey_camera.launch &

# Start the receive_file.py script in the background
python /home/uvify/catkin_ws/src/survey_mission/src/receive_file.py &

wait $ROSCORE_PID
