# setup
unitree_mujoco
unitree_sdk2_python

# check motor position
python3 Motor_Manager_origin.py

# Port
start/setup_ports.sh


# rl controller
python3 controllerV3_him_imu.py config/reddog_him.yaml

# launch IMU joystick Motor
ros2 launch reddog_hardware bringup.launch.py  




# IMU
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py 