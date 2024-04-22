#!/bin/bash

# Rebuild GISKARD workspace mounted from the host machine
cd ${GISKARD_WS}
catkin build
source ${GISKARD_WS}/devel/setup.bash

cd ${HOME}/blockly-playground

exec "$@"
