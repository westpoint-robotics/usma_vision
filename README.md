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

####Running Package
1. Open `pointgrey_camera.launch` and verify the serial number is correctly set for the `camera_serial` argument.
 - You can find the serial number by running `rosrun pointgrey_camera_driver list_cameras`
2. Run the launch file by typing `roslaunch usma_vision pointgrey_camera`
3. In order to see the camera feed open up rviz and add an image display.
 - Use `rostopic list` to find the camera topic you want to view
 - Enter the topic in the image display settings so that it subscribes to the camera feed

####Single Calibration
1. Install the image_pipeline package using `sudo apt-get install ros-indigo-image-pipeline`
2. Install the dependencies using `rosdep install --from-paths src --ignore-src --rosdistro=indigo -y`
3. Follow the tutorial at the link below.
 - `http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration`
 - Be sure to copy the calibration output from the terminal into a text file

###Dual Pointgrey

####Running Package
1. Open `dual_pointgrey_camera.launch` and verify the serial numbers are correct and are mapped to the right and left cameras from the perspective of the cameras.
2. Launch the file by typing `roslaunch usma_vision dual_pointgrey_camera.launch` 
 - This will automatically bring up rviz with a display for each camera feed

####Dual Calibration
1. Follow the single calibration instructions above and calibrate each camera individually.
2. Follow the tutorial at the link below.
 - `http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration`
 - The terminal command should look like this `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.1055 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left --approximate=0.01`
 - Approximate is necessary to allow for any slop due to the cameras not being synchronized
 - Be sure to copy the calibration output from the terminal into a text file

####Creating and Viewing Disparity Image and Pointcloud
1. Start up roscore
2. Launch the dual pointgrey package
 - `roslaunch usma_vision start_everything.launch`
3. To view the the disparity image you need to run the image_view script
 - `rosrun image_view stereo_view stereo:=stereo image:=image_rect`
 - It will take a little bit for it to start up after the windows appear
4. To view the pointcloud open Rviz and create a display for pointcloud2
