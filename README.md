# usma_vision
Camera configuration instructions

## Pointgrey cameras
###Driver Installation
1. Install [ROS pointgrey driver] (http://wiki.ros.org/pointgrey_camera_driver)
2. The following udev rules instructions are taken from this [post] (http://answers.ros.org/question/48244/unable-to-get-point-grey-usb-camera-work-in-ubuntu/).
 - Create this new udev rule: `sudo gedit /etc/udev/rules.d/10-pointgrey.rules`
 - Add this line to the file `SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="3300", GROUP="plugdev", SYMLINK+="blackfly", MODE:="0666"`
5. Restart udev by typing `sudo restart udev`
6. Add grub rule to /etc/default/grub
 - Get to the directory by typing `sudo nano /etc/default/grub`
 - Find `GRUB_CMDLINE_LINUX_DEFAULT`
 - Add `usbcore.usbfs_memory_mb=1000` to the list of commands
###Package Installation
1. In the terminal type `git clone https://github.com/westpoint-robotics/usma_vision.git` to clone it into your /src directory.
2. In your catkin workspace run `catkin_make`
Your package is now ready to be used!
###Single Pointgrey
1. Open `pointgrey_camera.launch` and verify the serial number is correctly set for the `camera_serial` argument.
 - You can find the serial number using the Pointgrey Flycapture2 software
2. Run the launch file by typing `roslaunch usma_vision pointgrey_camera`
3. In order to see the camera feed open up rviz and add an image display.
 - Use `rostopic list` to find the camera topic you want to view
 - Enter the topic in the image display settings so that it subscribes to the camera feed
###Dual Pointgrey
1. Open `dual_pointgrey_camera.launch` and verify the serial numbers are correct and are mapped to the right and left cameras from the perspective of the cameras.
2. Launch the file by typing `roslaunch usma_vision dual_pointgrey_camera.launch` 
 - This will automatically bring up rviz with a display for each camera feed
