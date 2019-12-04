#!/bin/bash
source /etc/ros/setup.bash
source /opt/ros/kinetic/setup.bash
source /home/nvidia/SureClean_UGV/devel/setup.bash
sleep 5
# Adjust the imu port latency
setserial /dev/ttyIMU0 low_latency
# Restart husky service to launch default husky nodes and roscore
sudo service husky-core restart
# wait until roscore is running
until rostopic list ; do sleep 1; done
sleep 3
# Kill default ekf
rosnode kill ekf_localization &
# Launch teleop node
roslaunch teleop_twist_joy teleop.launch &
