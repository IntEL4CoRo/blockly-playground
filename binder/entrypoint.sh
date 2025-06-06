#!/bin/bash
source ${ROS_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PWD}/launch/rvizweb_config/pr2_mujoco.json &

jupyter lab workspaces import  ${PWD}/binder/jupyterlab-workspace.json

# roslaunch ${PWD}/launch/pr2_mujoco.launch mujoco_suffix:=_headless &

exec "$@"
