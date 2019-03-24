# GPS_NAV 

This package provides a means for moving a husky using only GPS coordintes, and no other means of localization. The package makes use of the [husky_navigation](https://github.com/husky/husky/tree/kinetic-devel), [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/kinetic-devel), and [fake_localization](https://github.com/ros-planning/navigation/tree/kinetic-devel) packages to localize and control the husky. The actual gps navigation (gps_waypoint_nav) makes heavy use of the [outdoor_waypoint_nav](https://github.com/nickcharron/waypoint_nav) package. This package was developed in ROS Kinetic on an Ubuntu 16.04 system. This package was created for an academic project at Carnegie Mellon University.

## Installation and Setup

To setup the package, simply clone the repo into your catkin workspace and run `catkin_make`.

### Passing GPS Coordinates

GPS coordinates are read out of the text file 'gps_nav/src/gps_waypoint_nav/waypoint_files/points_sim.txt.' Each line is a set of GPS coordinates (latitude longitude)

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

3) In a new terminal, run `source devel/setup.bash` in the gps_nav package directory.

4) Launch the husky gps controller:
```
  $ roslaunch gps_waypoint_nav localization_run.launch
```