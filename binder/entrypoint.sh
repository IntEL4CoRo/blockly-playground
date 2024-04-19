#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=/home/${NB_USER}/blockly-playground/launch/pr2_mujoco.json &

roslaunch --wait /home/jovyan/blockly-playground/launch/stretch_standalone.launch &

exec "$@"
