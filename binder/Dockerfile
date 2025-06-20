FROM intel4coro/jupyter-ros2:humble-py3.10

USER ${NB_USER}
COPY --chown=${NB_USER}:users binder/install_hsr_sim.sh /home/${NB_USER}/
RUN chmod +x ~/install_hsr_sim.sh && \
    ~/install_hsr_sim.sh && \
    rm ~/install_hsr_sim.sh

WORKDIR /home/${NB_USER}/hsr_ros2_ws/
USER root
RUN apt update && \
    apt install -y ros-${ROS_DISTRO}-realtime-tools && \
    rosdep update && \
    rosdep install --from-paths . -y --ignore-src && \
    rosdep fix-permissions

USER ${NB_USER}
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source ~/hsr_ros2_ws/install/setup.bash" >> ~/.bashrc
RUN pip install lxml matplotlib

COPY --chown=${NB_USER}:users . /home/${NB_USER}/blockly-playground

# --- Entrypoint --- #
COPY --chown=${NB_USER}:users entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]