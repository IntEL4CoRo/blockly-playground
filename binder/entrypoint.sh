#!/bin/bash
source ${ROS_WS}/devel/setup.bash

roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${PWD}/launch/rvizweb_config/pr2_mujoco.json &

roslaunch --wait iai_apartment apartment_bringup.launch &

jupyter lab workspaces import  ${PWD}/binder/jupyterlab-workspace.json

exec "$@"
