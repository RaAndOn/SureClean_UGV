#!/bin/bash
source /etc/ros/setup.bash
source /opt/ros/kinetic/setup.bash
source /home/nvidia/husky_kinetic_ws/devel/setup.bash
sleep 10
# Restart husky service to launch default husky nodes and roscore
sudo service husky-core restart
# wait until roscore is running
until rostopic list ; do sleep 1; done
sleep 2
# Kill deafualt imu node to make room for actual node
rosnode kill /um6_driver
rosnode kill /imu_manager
rosnode kill /imu_data_transformer
rosnode kill /imu_filter
# Wait to be sure it has taken effect
sleep 2
# launch imu nodes
roslaunch husky_bringup um7.launch & 
# Launch teleop node
roslaunch teleop_twist_joy teleop.launch &