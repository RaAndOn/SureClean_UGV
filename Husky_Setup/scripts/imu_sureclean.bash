#!/bin/bash
# Check that a valid location has been provided and set the MAG variable
MAG=$1
if [ "$MAG" != "true" ] && [ "$MAG" != "false" ]
then
	echo "Invalid first param LOCATION provided! Please pass either 'true' or 'false'"
    exit 1
fi
# Be sure ROS Core is running
until rostopic list ; do sleep 1; done
# Adjust the imu port latency
setserial /dev/ttyIMU0 low_latency
sleep 3
# Kill existing nodes
rosnode kill /um7_driver
rosnode kill /imu_data_transformer
rosnode kill /imu_manager
rosnode kill /imu_filter
# Launch IMU Nodes
roslaunch sureclean_utils um7_sureclean.launch id:=1 use_mag:=$MAG &
sleep 5
echo "IMU Nodes launched!"
# Reset imu ekf and gyros
rosservice call /imu_um7/reset "set_mag_ref: true"
rosservice call /imu_um7/reset "zero_gyros: true"
rosservice call /imu_um7/reset "reset_ekf: true"
echo "IMU Zeroed!"
