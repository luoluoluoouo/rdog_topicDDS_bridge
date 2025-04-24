#!/bin/bash


python3 /home/csl/rdog/reddog_ros2_ws/src/redDog_RL_Him/controller/rl_quadruped_controller/rl_quadruped_controller/controllerV3_him_imu.py /home/csl/rdog/reddog_ros2_ws/src/redDog_RL_Him/controller/rl_quadruped_controller/rl_quadruped_controller/config/reddog_him.yaml #&> ./controller.log

sleep 10
source /home/csl/rdog/reddog_ros2_ws/install/setup.bash
ros2 launch reddog_hardware bringup.launch.py #&> ./launch.log