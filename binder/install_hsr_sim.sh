#!/bin/bash

mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
git clone -b humble https://github.com/hsr-project/gazebo_ros2_control.git
git clone -b humble https://github.com/hsr-project/hsrb_controllers.git
git clone -b humble https://github.com/hsr-project/hsrb_common.git
git clone -b humble https://github.com/hsr-project/hsrb_drivers.git
git clone -b humble https://github.com/hsr-project/hsrb_launch.git
git clone -b humble https://github.com/hsr-project/hsrb_manipulation.git
git clone -b humble https://github.com/hsr-project/hsrb_rosnav.git
git clone -b humble https://github.com/hsr-project/hsrb_simulator.git
git clone -b humble https://github.com/hsr-project/hsr_common.git
git clone -b humble https://github.com/hsr-project/hsrb_teleop.git
git clone -b humble https://github.com/hsr-project/tmc_gazebo.git
git clone -b humble https://github.com/hsr-project/tmc_teleop.git
git clone -b humble https://github.com/hsr-project/tmc_common.git
git clone -b humble https://github.com/hsr-project/tmc_common_msgs.git
git clone -b humble https://github.com/hsr-project/tmc_drivers.git
git clone -b humble https://github.com/hsr-project/tmc_database.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_base.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_planner.git
git clone -b humble https://github.com/hsr-project/tmc_realtime_control.git
git clone -b humble https://github.com/hsr-project/tmc_voice.git
git clone -b humble https://github.com/hsr-project/tmc_navigation.git
rm -rf hsrb_launch/hsrb_robot_launch
rm -rf hsrb_simulator/hsrb_rviz_simulator
rm -rf tmc_drivers/tmc_pgr_camera
