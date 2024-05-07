FROM intel4coro/base-notebook:20.04-noetic-vnc

# Building Custom bullet3 from source with Python 3.10
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

# Initiate GISKARD ROS workspace
ENV ROS_WS=/home/${NB_USER}/workspace/ros
ENV GISKARD_WS=/home/${NB_USER}/workspace/giskard_ws
RUN mkdir -p ${GISKARD_WS}/src
WORKDIR ${GISKARD_WS}/src
COPY --chown=${NB_USER}:users binder/noetic.rosinstall /home/${NB_USER}/noetic.rosinstall
RUN  wstool init \
  && wstool merge /home/${NB_USER}/noetic.rosinstall \
  && wstool update

# Install dependencies
WORKDIR  ${GISKARD_WS}
USER root
RUN rosdep update && apt update && apt dist-upgrade -y \
  && rosdep install -y --ignore-src --from-paths ./ -r \
  && rosdep fix-permissions
# Install missing apt packages for building ROS package 'qpoases'
RUN apt install -y libglfw3 libglfw3-dev
USER ${NB_USER}
RUN pip install -r src/giskardpy/requirements.txt

# Install missing python packages (should be included in src/giskardpy/requirements.txt)
RUN pip install termcolor pydot triangle cython~=0.19

# A Workaround for fixing the issue of RvizWeb loading meshes with "file:///" path instead of "package:///".
COPY --chown=${NB_USER}:users binder/me ${ROS_WS}/src/me
RUN cd ${ROS_WS} && catkin build

# Build GISKARD workspace
WORKDIR  ${GISKARD_WS}
RUN catkin init
RUN catkin config --extend ${ROS_WS}/devel
RUN catkin build
RUN echo "source ${GISKARD_WS}/devel/setup.bash" >> /home/${NB_USER}/.bashrc

RUN npm i -g yarn

# Copy the giskard tmp meshes to skip the converting step
COPY --chown=${NB_USER}:users binder/giskard_decomposed_obj ${GISKARD_WS}/src/giskardpy/tmp
# Copy contents of the repo into the image
COPY --chown=${NB_USER}:users . /home/${NB_USER}/blockly-playground
WORKDIR /home/${NB_USER}/blockly-playground
RUN git config --global --add safe.directory ${PWD}
# Link the ROS workspace directory to the working directory
RUN ln -s ${GISKARD_WS} ${PWD}/GISKARD_WS

# Install blockly jupyterlab extension from source
RUN cd jupyterlab-blockly && pip install -e .

# Install blockly jupyterlab extension from built package
# RUN pip install https://raw.githubusercontent.com/yxzhan/jupyterlab-blockly/dev/dist/jupyterlab_blockly-0.3.0a4.tar.gz

# jupyterlab interface configuration
COPY --chown=${NB_USER}:users binder/webapps.json ${ROS_WS}/src/rvizweb/webapps/app.json
COPY --chown=${NB_USER}:users binder/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["start-notebook.sh"]
