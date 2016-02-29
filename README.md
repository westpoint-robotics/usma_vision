# usma_vision
Camera configuration instructions

### Pointgrey cameras
1. Install [ROS pointgrey driver] (http://wiki.ros.org/pointgrey_camera_driver)
2. The follow udev rules intstructions are taken from this [post] (http://answers.ros.org/question/48244/unable-to-get-point-grey-usb-camera-work-in-ubuntu/).
3. Create this new udev rule: `sudo gedit /etc/udev/rules.d/10-pointgrey.rules`
4. Add this line:
 - `SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="3300", GROUP="plugdev", SYMLINK+="blackfly", MODE:="0666"`
5. Restart: 
 - `sudo restart udev`
