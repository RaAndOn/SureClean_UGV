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
rosrun reach_ros_node nmea_tcp_driver &
sleep 2
# Reset imu ekf and gyros
rosservice call /imu_um7/reset "set_mag_ref: true"
rosservice call /imu_um7/reset "zero_gyros: true"
rosservice call /imu_um7/reset "reset_ekf: true"
# Run Setup Fused Odometry
rosnode kill ekf_localization &
roslaunch robot_localization sureclean_navsat_ekf.launch &
# Launch teleop node
roslaunch teleop_twist_joy teleop.launch &
