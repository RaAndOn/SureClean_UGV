Detect the IMU via ls USB

Check the usb port via ls -l dev/ttyACM*

Then use the following two commands:

sudo usermod -a -G dialout $USER

sudo chmod a+rw /dev/ttyACM0  (The last number can be different)



