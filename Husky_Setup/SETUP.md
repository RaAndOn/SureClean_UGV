# Husky

## Getting Started

There are two ways to setup the Husky software. Either you can do a fresh Ubuntu install using a specially made Clearpath ISO, or you can manually install the software on your Ubuntu machine (**The Ubuntu Machine must be at least 16.04 and have apt-get**, so Ubuntu Core will not work using this procedure). These instructions will cover both installation instructions.

### [Installation on an existing Ubuntu System](http://wiki.ros.org/husky_bringup/Tutorials/Install%20Husky%20Software%20%28Advanced%29)

1) Setup your robot PC to with Ubuntu Xenial
2) Setup ROS [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
3) Setup the [Clearpath Robotics packages repository](http://packages.clearpathrobotics.com) 
4) Install the Husky packages: 
```
$ sudo apt-get install ros-kinetic-husky-robot
```
5) Install the udev rules that come with Husky:
```
 $ sudo cp $(rospack find husky_bringup)/udev/*.rules /etc/udev/rules.d
```
6) Create a robot-wide setup file, such as /etc/ros/setup.bash (sample below): 
```
# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash
 
# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=br0
# Insert extra platform-level environment variables here. The six hashes below are a marker
# for scripts to insert to this file.
 
######
# Pass through to the main ROS workspace of the system.
source /opt/ros/kinetic/setup.bash
```
7) Source this robot-wide setup file in your ~/.bashrc file: 
```
source /etc/ros/setup.bash
```
8) Refresh your environment by executing: 
```
$ . ~/.bashrc
```
9) Create a robot_upstart job to start the Husky software each time the robot PC is booted. This command will create the job, and copy the base.launch file to the /etc/ros/kinetic/husky-core.d/ directory: 
```
$ rosrun robot_upstart install husky_base/launch/base.launch --job husky-core --setup '/etc/ros/setup.bash'
```
10) To start launch files for additional standard Husky components, we can augment the husky-core job. Run the following command to add any launch files in the husky_bringup/launch/um6_config/ directory to the job:
```
$ rosrun robot_upstart install husky_bringup/launch/um6_config --job husky-core --augment
```

**Warning** ROS Kinetic is written to run with python 2.7, if you installed Anaconda or have modified your default python settings in some way your computer might try to run this files using python 3. You can check this using the command `python --version`


### [Installation using Clearpath ISO](http://www.clearpathrobotics.com/assets/guides/husky/InstallHuskySoftware.html)
1) Download the appropriate [Kinetic Husky ISO](http://packages.clearpathrobotics.com/stable/images/latest/kinetic-husky/) image for your platform (32 bit - i386, 64 bit - amd64).

2) Setup an install USB

3) Connect your robot PC to wired internet access, a keyboard, and a monitor. Make sure that the PC is connected to shore power, or the Husky battery is either fully charged.

**Warning**
> The next step wipe your robot’s hard drive, so make sure you have that image backed up on another system!

4) Boot your robot PC from the USB drive, and let installer work it’s magic.
5) The setup process will be automated, and may take a long time depending on the speed of your internet connection.
6) Once the setup process is complete, the PC will turn off. Please unplug the USB drive and turn the PC back on.
7) On first boot, the username will be administrator and the password will be clearpath.
8) Please follow the configuration instructions on the screen. If the computer reboots, wait for the PC to boot to the login screen, and re-enter the login credentials.
9) Once the computer configuration is complete, you may use passwd utility to change administrator account password.
11) Configure the IMU
a) Modify the `/etc/ros/setup.bash` to add the following lines 
```
export HUSKY_IMU_PORT=/dev/ttyUSB0
export HUSKY_IMU_RPY="0.0 3.1416 0.0"
```
These will 1) set up the husky to look for IMU data at this USB port and 2) override the default husky IMU orientation to the one we are using on project Sureclean. *NOTE:* `/dev/ttyUSB0` is the port on which IMU data comes in *at time of writing,* confirm this is still the case before running this.

10) To setup a factory-standard Husky robot, ensure all your peripherals are plugged in, and run the following command:
```
$ rosrun husky_bringup install
```
The install script will configure a husky-core upstart service, that will bring up the base Husky launchfiles on boot. The script will also detect any standard peripherals (IMU, GPS, etc.) you have installed, and add them the service.

## Testing base configuration

1) To test your configuration, start the background service with the following command:
```
$ sudo service husky-core start
```
    
2) The COMM light on your Husky should go from red to green. You can check that the service has started correctly by checking the logs:
```
$ sudo tail /var/log/upstart/husky-core.log -n 30
```
3) Your husky should now be accepting commands from your joystick. The service will automatically start each time you boot your Husky's PC. 

4) If you are not seeing IMU data published on any topic, run `roslaunch husky_bringup um7.launch` and check all IMU topics again. 
If this starts publishing IMU Data be sure to perform the `Post Install Configuration`, which should fix the problem. Or checkout the 
IMU package [website](http://wiki.ros.org/um7)


## [Diagnostics](http://www.clearpathrobotics.com/assets/guides/husky/InterfacingWithHusky.html)

To get diagnostic information on the husky, install rqt by running the [following](http://wiki.ros.org/rqt/UserGuide/Install/Groovy):

1) Standard packages (rqt's core library + common plugins) can be installed by: 
```
$ sudo apt-get install ros-%YOUR_ROS_DISTRO%-rqt ros-%YOUR_ROS_DISTRO%-rqt-common-plugins`
```
2) Additionally, you can also install rqt_robot_plugins that provide features to be used when interacting with robots:

```
$ sudo apt-get install ros-%YOUR_ROS_DISTRO%-rqt-robot-plugins
$ sudo apt-get install ros-%YOUR_ROS_DISTRO%-rqt-pr2-dashboard (only when you need PR2 dashboard feature) 
```

3) If you only want to install a specific rqt plugin (rqt_moveit for instance):
```
$ sudo apt-get install ros-%YOUR_ROS_DISTRO%-rqt-moveit
```

Once rqt is installed you can activate rqt by running:
```
$ rosrun rqt_runtime_monitor rqt_runtime_monitor
```

## Post Install Configuration

### husky_startup.sh

The file `husky_startup.sh` is a shell script which is meant to be run at startup to setup the husky including:

1) Starting the roscore/husky_core
2) Starting the imu
3) Enabling the teleop

When you copy the script into place, don't forget to make it executable and owned by root:
```
sudo chmod +x /etc/init.d/husky_startup.sh
sudo chown root:root /etc/init.d/husky_startup.sh
```
Once you have installed that, you can set it to start automatically on boot with:
```
sudo update-rc.d husky_startup.sh defaults
sudo update-rc.d husky_startup.sh enable
```

### Automatically Connect to TP-Link

To automatically connect to TP-Link over CMU-Device on startup, you must give TP-Link a higher
'autoconnect priority' using the nmcli tool. You can do so by running the following command:
```
$ nmcli connection modify TP-Link_E7E8 connection.autoconnect-priority 10
```
Changing `TP-Link_E7E8` to whatever the appropriate network name is.