#!/bin/bash
source ${GISKARD_WS}/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=/home/${NB_USER}/blockly-playground/launch/stretch.json &

roslaunch --wait /home/jovyan/blockly-playground/launch/stretch_standalone.launch &

# Install the jupyterlab-blockly extension in development mode
# cd jupyterlab-blockly
# pip install -e .
# jupyter labextension develop . --overwrite
# ln -s ${HOME}/blockly-playground/jupyterlab-blockly/packages/blockly/src/giskard.ts ${HOME}/blockly-playground/examples/blockly.ts
# npm run watch &

# cd ${HOME}/blockly-playground

exec "$@"
