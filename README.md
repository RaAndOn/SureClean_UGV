# SureClean_UGV
All content related to the UGV for Project SureClean. This repo is built to act as a catkin workspace for Project Sureclean's UGV.
As such all ROS Packages should be added directly to the src folder, for easy tracking.

## Husky Folder

The `Husky` folder at the root of the project contains information pertaining to setting up a Husky to be used for the Sureclean project.
The README in the folder contains more detailed setup instructions.

## Installation and Setup

To setup any packages included in SureClean_UGV, simply clone the repo into home directory and run `catkin_make`. 

Then run `source devel/setup.bash`

*Be sure to add* `source /home/nvidia/SureClean_UGV/devel/setup.bash` *to the end of the UGV's `~/.bashrc` file*


## Simulation

To test the code in simulation, first install husky_gazebo following [these instructions](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky).

1) If husky gazebo is installed, in a terminal set an environmental variable HUSKY_GAZEBO_DESCRIPTION:
```
  $ export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
```
2) Run one of the provided husky simulations, being sure to specify `laser_enabled:=false` to disable the laser range finder:
i) Simulate Husky in an empty world
```
  $ roslaunch husky_gazebo husky_empty_world.launch laser_enabled:=false
```
ii)
```
  $ roslaunch husky_gazebo husky_playpen.launch laser_enabled:=false
```

3) In a new terminal, run `source devel/setup.bash` in the SureClean_UGV package directory.

4) Launch the husky gps controller:
```
  $ roslaunch sureclean_utils sureclean.launch sim:=true
```

# Debugging

## Logging

For ease of use, the nodes are deployed at time of UGV start-up so all logging levels are set to "output=log".
Therefore, when adding print statements during debugging, one should use `rqt_console` view any outputs.

## Debug Building

To use most debugging features, one must build the package with the following option set:
```
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```
