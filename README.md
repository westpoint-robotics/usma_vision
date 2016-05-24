# usma_vision
Camera configuration instructions

### Pointgrey cameras
1. Install [ROS pointgrey driver] (http://wiki.ros.org/pointgrey_camera_driver)
2. The following udev rules instructions are taken from this [post] (http://answers.ros.org/question/48244/unable-to-get-point-grey-usb-camera-work-in-ubuntu/).
 - Create this new udev rule: `sudo gedit /etc/udev/rules.d/10-pointgrey.rules`
 - Add this line to the file `SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="3300", GROUP="plugdev", SYMLINK+="blackfly", MODE:="0666"`
5. Restart udev by typing `sudo restart udev`
6. Add grub rule to /etc/default/grub
 - Get to the directory by typing 'sudo nano /etc/default/grub'
 - Find `GRUB_CMDLINE_LINUX_DEFAULT`
 - Add `usbcore.usbfs_memory_mb=1000` to the list of commands
