#!/bin/bash
source ${GISKARD_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=/home/${NB_USER}/blockly-playground/launch/stretch.json &

roslaunch --wait /home/jovyan/blockly-playground/launch/stretch_standalone.launch &

# cd jupyterlab-blockly
# pip install -e .
# jupyter labextension develop . --overwrite
# npm run watch &
# cd ${HOME}/blockly-playground

exec "$@"
