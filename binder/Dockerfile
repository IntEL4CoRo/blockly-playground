FROM intel4coro/base-notebook:20.04-noetic-vnc

# Building Custom bullet3 from source with Python 3.10 (GISKARD required)
WORKDIR /home/${NB_USER}
RUN git clone https://github.com/pybind/pybind11.git \
  && cd pybind11 \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3 -DPYBIND11_TEST=OFF
USER root
RUN cd pybind11/build && make install
USER ${NB_USER}

RUN git clone https://github.com/SemRoCo/bullet3.git
COPY --chown=${NB_USER}:users binder/build_cmake_pybullet_3.10_double.sh /home/${NB_USER}/bullet3/
RUN cd bullet3 && ./build_cmake_pybullet_3.10_double.sh
ENV PYTHONPATH=${PYTHONPATH}:/home/${NB_USER}/bullet3/build_cmake/better_python:/home/${NB_USER}/bullet3/examples/pybullet

# Initiate ROS workspace
ENV ROS_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${ROS_WS}/src
# A Workaround for fixing the issue of RvizWeb loading meshes with "file:///" path instead of "package:///".
COPY --chown=${NB_USER}:users binder/me ${ROS_WS}/src/me
COPY --chown=${NB_USER}:users binder/noetic.rosinstall /home/${NB_USER}/noetic.rosinstall
RUN wstool merge /home/${NB_USER}/noetic.rosinstall && wstool update

# Clone repos with ssh url in .gitmodules
RUN git clone https://github.com/maltehue/mujoco_robots.git \
  && cd mujoco_robots \
  # regular expression converts ssh urls to https urls in .gitmodules
  && perl -i -p -e 's|git@(.*?):|https://\1/|g' .gitmodules \
  && git submodule sync \
  && git submodule update --init --recursive \
  && rm -rf .git
RUN git clone https://github.com/HoangGiang93/mujoco_world.git \
  && cd mujoco_world \
  && perl -i -p -e 's|git@(.*?):|https://\1/|g' .gitmodules \
  && git submodule sync \
  && git submodule update --init --recursive \
  # replace the apartment models with the latest ones
  && git clone https://github.com/Multiverse-Framework/Multiverse-World.git \
  && rm -rf iai_maps/iai_apartment \
  && mv Multiverse-World/iai_apartment iai_maps/ \
  && rm -rf Multiverse-World .git

# To fix some files can not be found in rvizweb
ARG MUJOCO_WORLD_PATH=${ROS_WS}/src/mujoco_world/mujoco_world
RUN mkdir ${MUJOCO_WORLD_PATH}/mujoco_world && \
    ln -s ${MUJOCO_WORLD_PATH}/model ${MUJOCO_WORLD_PATH}/mujoco_world/model

# Install ROS dependencies
WORKDIR  ${ROS_WS}
USER root
RUN rosdep update --include-eol-distros && apt update \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
# Install missing apt packages to build ROS package 'qpoases'
RUN apt install -y libglfw3 libglfw3-dev
USER ${NB_USER}
RUN pip install -r src/giskardpy/requirements.txt

# Install missing python packages
RUN pip install \
    termcolor \
    pydot \
    triangle \
    cython~=0.19 \
    py_trees==0.7.6 \
    rospy-message-converter

# Build ROS workspace
WORKDIR  ${ROS_WS}
# Can not build package iai_pr2_donbot and it is not used
RUN rm -rf src/iai_pr2/iai_pr2_donbot
RUN rm -rf src/iai_pr2/iai_pr2_sim/CATKIN_IGNORE
RUN catkin build

# Install blockly jupyterlab extensions
RUN npm install -g yarn && \
    pip install git+https://github.com/yxzhan/jupyterlab-blockly.git@dev && \
    rm -rf /home/${NB_USER}/.cahche/yarn

# Copy the giskard tmp meshes to skip the converting step
COPY --chown=${NB_USER}:users binder/giskard_decomposed_obj ${ROS_WS}/src/giskardpy/tmp
# Copy contents of the repo into the image
COPY --chown=${NB_USER}:users . /home/${NB_USER}/blockly-playground
WORKDIR /home/${NB_USER}/blockly-playground
RUN git config --global --add safe.directory ${PWD}
# Link the ROS workspace directory to the working directory
RUN ln -s ${ROS_WS} ${PWD}/ROS_WS
# Overrides the kitchen and apartment xml files
ARG MJ_WORLD_PATH=${ROS_WS}/src/mujoco_world/mujoco_world/model
RUN rm ${MJ_WORLD_PATH}/iai_apartment/iai_apartment_with_window4.xml
RUN ln -s ${PWD}/launch/mujoco_config/iai_apartment_with_window4.xml ${MJ_WORLD_PATH}/iai_apartment/iai_apartment_with_window4.xml
RUN rm ${MJ_WORLD_PATH}/iai_kitchen/iai_kitchen_python.xml
RUN ln -s ${PWD}/launch/mujoco_config/iai_kitchen_python.xml ${MJ_WORLD_PATH}/iai_kitchen/iai_apartment_with_window4.xml

# Setup Ipython startup script (Open Rviz)
RUN ipython profile create && \
    ln -s /home/${NB_USER}/blockly-playground/examples/00-first.py \
    /home/${NB_USER}/.ipython/profile_default/startup/00-first.py

# Disable RVIZ EOL warning
ENV DISABLE_ROS1_EOL_WARNINGS=1
# jupyterlab launcher configuration
RUN rm ${ROS_WS}/src/rvizweb/webapps/app.json && \
    ln -s ${PWD}/binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]
