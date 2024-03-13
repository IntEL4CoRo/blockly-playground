FROM intel4coro/base-notebook:20.04-noetic

# Initiate ROS workspace
ENV ROS_WS=/home/${NB_USER}/workspace/ros
WORKDIR ${ROS_WS}/src
# A Workaround for fixing the issue of RvizWeb loading meshes with "file:///" path instead of "package:///".
COPY --chown=${NB_USER}:users binder/me ${ROS_WS}/src/me
COPY --chown=${NB_USER}:users binder/noetic.rosinstall /home/${NB_USER}/noetic.rosinstall
RUN wstool merge /home/${NB_USER}/noetic.rosinstall \
  && wstool update
# Clone repos with ssh url in .gitmodules
RUN git clone https://github.com/maltehue/mujoco_robots.git \
  && cd mujoco_robots \
  # regular expression converts ssh urls to https urls in .gitmodules
  && perl -i -p -e 's|git@(.*?):|https://\1/|g' .gitmodules \
  && git submodule sync \
  && git submodule update --init --recursive
RUN git clone https://github.com/HoangGiang93/mujoco_world.git \
  && cd mujoco_world \
  && perl -i -p -e 's|git@(.*?):|https://\1/|g' .gitmodules \
  && git submodule sync \
  && git submodule update --init --recursive
# To fix some files can not be found in rvizweb
ARG MUJOCO_WORLD_PATH=${ROS_WS}/src/mujoco_world/mujoco_world
RUN mkdir ${MUJOCO_WORLD_PATH}/mujoco_world && \
    ln -s ${MUJOCO_WORLD_PATH}/model ${MUJOCO_WORLD_PATH}/mujoco_world/model

# Install dependencies
WORKDIR  ${ROS_WS}
USER root
RUN rosdep update && apt update && apt dist-upgrade -y \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
# Install missing apt packages to build ROS package 'qpoases'
RUN apt install -y libglfw3 libglfw3-dev
USER ${NB_USER}
RUN pip install -r src/giskardpy/requirements.txt

# Install missing python packages (should be included in src/giskardpy/requirements.txt)
RUN pip install termcolor pydot triangle cython~=0.19

# Building Custom bullet3 from source with Python 3.10 (in a clumsy way)
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

# Build ROS workspace
WORKDIR  ${ROS_WS}
# Can not build package iai_pr2_donbot and it is not used
RUN rm -rf src/iai_pr2/iai_pr2_donbot
RUN catkin build

# Install blockly jupyterlab extensions
RUN npm install -g yarn
RUN pip install git+https://github.com/yxzhan/jupyterlab-blockly.git@dev
RUN pip install jupyterlab-git~=0.44.0

# Copy the giskard tmp meshes to skip the converting step
COPY --chown=${NB_USER}:users binder/giskard_decomposed_obj ${ROS_WS}/src/giskardpy/tmp
# Copy contents of the repo into the image
COPY --chown=${NB_USER}:users . /home/${NB_USER}/blockly-playground
WORKDIR /home/${NB_USER}/blockly-playground
RUN git config --global --add safe.directory ${PWD}
# Link the ROS workspace directory to the working directory
RUN ln -s ${ROS_WS} ${PWD}/ROS_WS
# Overrides the kitchen and apartment xml files
COPY --chown=${NB_USER}:users ./launch/mujoco_config/iai_apartment_with_window4.xml ${ROS_WS}/src/mujoco_world/mujoco_world/model/iai_apartment/
COPY --chown=${NB_USER}:users ./launch/mujoco_config/iai_kitchen_python.xml ${ROS_WS}/src/mujoco_world/mujoco_world/model/iai_kitchen/

# jupyterlab interface configuration
COPY --chown=${NB_USER}:users binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json
COPY --chown=${NB_USER}:users binder/jupyter-settings.json /opt/conda/share/jupyter/lab/settings/overrides.json
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]

# Update the rvizweb
RUN cd ${ROS_WS}/src/rvizweb && git pull
