Detect the IMU via ls USB

Check the usb port via ls -l dev/ttyACM*

Then use the following two commands:

sudo usermod -a -G dialout $USER

sudo chmod a+rw /dev/ttyACM0  (The last number can be different)


Additional Steps to be followed to Flash the IMU 
https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide/all

Note: the additional steps need Board Manager option in Arduino that is not available on Linux. Therefore, use Mac/Windows to execute the 'additional steps'
