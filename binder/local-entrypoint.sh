#!/bin/bash
# Start rvizweb
# source ${ROS_WS}/devel/setup.bash
# roslaunch --wait rvizweb rvizweb.launch config_file:=/home/${NB_USER}/blockly-playground/launch/stretch.json &

# Rebuild GISKARD workspace from the host machine
cd ${GISKARD_WS}
catkin build
source ${GISKARD_WS}/devel/setup.bash

cd ${HOME}/blockly-playground

exec "$@"
