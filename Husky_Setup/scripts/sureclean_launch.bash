#!/bin/bash
MAG=$1
COVERAGE=$2
# Check that a valid option for use of mag_updates was provided
if [ "$MAG" != "true" ] && [ "$MAG" != "false" ]
then
	echo "Invalid first param MAG provided! Please pass either 'true' or 'false'"
    exit 1
fi
# Check that a valid option for use of coverage planner was provided
if [ "$COVERAGE" != "true" ] && [ "$COVERAGE" != "false" ]
then
	echo "Invalid second param COVERAGE provided! Please pass either 'true' or 'false'"
    exit 1
fi
# Launch IMU
imu_sureclean.bash $MAG
# Launch Sureclean
roslaunch sureclean_utils sureclean.launch coverage:=$COVERAGE
