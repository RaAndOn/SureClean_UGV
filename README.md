# SureClean_UGV
All content related to the UGV for Project SureClean. This repo is built to act as a catkin workspace for Project Sureclean's UGV.
As such all ROS Packages should be added directly to the src folder, for easy tracking.

## Husky Folder

The `Husky` folder at the root of the project contains information pertaining to setting up a Husky to be used for the Sureclean project.
The README in the folder contains more detailed setup instructions.

## Installation and Setup

To setup any packages included in SureClean_UGV, simply clone the repo into home directory and run `catkin_make`. 

Then run `source devel/setup.bash`

*Be sure to add* `source ~/SureClean_UGV/devel/setup.bash` *to the end of the UGV's `~/.bashrc` file*
