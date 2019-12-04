# kinect-ROS
This repository is just for work with Microsoft Kinect and ROS

Here is a short code which you can use for integration of Microsoft Kinect v1 with ROS-Kinetic on Ubuntu 16.04. In order to use Micrososoft Kinect with ROS kinectic is needed recompile OpenCV and use OPENNI for open device. Once you have installed ROS-kinetic, you need to Install Kinect Drivers, Install OPENNI.

The script provided the OpenCV 3.31 download, OpenCV_contribs3.3.1, CUDA 8.0.6.1, CudNN 5.1.

After that compile OpenCV with CUDA, CudNN, OPENNI. Finally, replace and make symbolic links ROS openCV with the compiled libraries.

### Requirements:

- Ubuntu 16.04
- ROS Kinetic

### Use:

- Run `$ ./toInstall.sh`
	- It download and install CUDA 8.0
	- It download and install CudNN 5.1
	- It download  OpenCV_contribs 3.3.1
	- It download and compile OpenCV 3.3.1
	- Replace ROS opencv libs with recently compiled libraries.
- Move into the directories to "catkin_ws"
- Compile `$catkin_make`

### Test:
- `$ roscore` in one terminal
- `$ rosrun kinect_man kinect_man_node` in other terminal.
- Make sure that you have sourced the catkin workspace `$ source \path_to\kinetic-ROS\ROS\catkin_ws\build\setup.bash`
