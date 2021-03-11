# bfft_formula-student_driverless

<p align="center">
  <a href="https://blackforestformula.hs-offenburg.de/">
    <img alt="BFFT_Logo" title="BFFT" src="https://scontent-frt3-1.xx.fbcdn.net/v/t1.0-9/69419451_117866062911797_4569414645357477888_o.jpg?_nc_cat=107&ccb=1-3&_nc_sid=973b4a&_nc_ohc=b5rqMomf8_AAX8x_CMD&_nc_ht=scontent-frt3-1.xx&oh=7ab30784f93fdf5ad846196156f856e6&oe=606D20C4" width="1000">
  </a>
</p>

# Black Forest Formula Team - Formula Student Driverless 2021
<!-- Template Readme:https://github.com/gitpoint/git-point#readme -->

This repository lays the foundation for the future developments of autonomous features of the Black Forest Formula Team located in Offenburg. You can find an overview to get started in this ReadMe, for more information we suggest to refer to the [Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki) you can find in this Repo.
____________________


## Repository organisation
<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->


- [Introduction](#introduction)
- [Installation](#installation)
  - [Preconditions & Current Setup](#preconditions--current-setup)
  - [Additional and mandatory Libraries and Tools](#additional-and-mandatory-libraries-and-tools)
  - [Install ROS Melodic](#install-ros-melodic)
  - [Setup ROS Catkin-Workspace and Download needed Packages](#setup-ros-catkin-workspace-and-download-needed-packages)
    - [Clone packages into src folder inside workspace](#clone-packages-into-src-folder-inside-workspace)
  - [Build Process](#build-process)
- [Getting started](#getting-started)
  - [Intel Camera - Setup](#intel-camera---setup)
  - [CAN Bus of Jetson AGX Xavier - Setup](#can-bus-of-jetson-agx-xavier---setup)
    - [Software Setup](#software-setup)
    - [Hardware Setup](#hardware-setup)
  - [Connecting IMU and sensors via CAN bus](#connecting-imu-and-sensors-via-can-bus)
  - [How to start recording of Data into ROSBAGS](#how-to-start-recording-of-data-into-rosbags)
- [Usage examples](#usage-examples)
  - [Convenience scripts for AGX ROS remote control via Ethernet/Wifi](#convenience-scripts-for-agx-ros-remote-control-via-ethernetwifi)
  - [Start autonomous system from Windows laptop](#start-autonomous-system-from-windows-laptop)
  - [Stop autonomous system from Windows laptop](#stop-autonomous-system-from-windows-laptop)
  - [Convert ROSBAG to CSV file](#convert-rosbag-to-csv-file)
  - [Copy CSV files with CAN-Data to Windows Laptop](#copy-csv-files-with-can-data-to-windows-laptop)
  - [Display Data in Tableau](#display-data-in-tableau)
- [Features Datavisualization](#features-datavisualization)
- [Code Repository Conventions](#code-repository-conventions)
  - [Python](#python)
  - [ROS Python](#ros-python)
- [ROS naming conventions](#ros-naming-conventions)
  - [Work packages:](#work-packages)
  - [ROS packages:](#ros-packages)
  - [ROS nodes](#ros-nodes)
  - [ROS topics](#ros-topics)
  - [ROS messages](#ros-messages)
- [Feedback](#feedback)
- [Sponsors](#sponsors)
- [Acknowledgments](#acknowledgments)
- [Our Developers](#our-developers)
- [Release History](#release-history)
- [Meta](#meta)
- [Contributing](#contributing)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->
____________________
## Introduction
This setup and code implementation has three main purposes:
1. Visualize Data of our first electric racecar when doing testdrives and real races as close to the run as possible (best of course would be real-time)
2. Record all relevant incoming data while doing a test run for simulation purposes later on (be it for the controll system, temp simulation, object detection or others)
3. Build the foundation to push our car to one day drive with autonomous features

Therefore, we decided to use ROS (and soon ROS2) for implementing the given goals as stated above. Right now we are almost ready to fulfill goal Nr. 1 as well as goal Nr. 2. Until goal Nr. 3 there is still quite a way to got. Our progress of the software setup and how it plays together with our hardware can be seen in the image below.

<p align="center">
  <img src = "https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/blob/master/img/ROS_Architecture_20210311.svg" width=1000>
</p>


____________________
## Installation

### Preconditions & Current Setup
Software
* Ubuntu 18.04 LTS
* Python 3.X
* ROS 1 Melodic
* JetPack 4.4
Hardware
* [NVIDIA Jetson AGX Xavier](https://elinux.org/Jetson_AGX_Xavier) with 32 GB RAM, upgraded 512 GB SSD and upgraded Intel Wifi 8265 
* [Intel D455 Cameras (x2)](https://www.intelrealsense.com/depth-camera-d455/)
* [Genesys Adma Slim IMU](https://www.genesys-offenburg.de/en/products/adma-slim-mini-gnssinertial-system/)
* [Some kind of CAN Transceiver and DB9 connectors]()
* 

### Additional and mandatory Libraries and Tools
* [Bagpy for Python 3](https://pypi.org/project/bagpy/)
```
pip3 install bagpy
```
* [Realsense Kernel Patch - scroll down to: Building from Source using Native Backend](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
* [Open_cv for Tegra AGX Xavier - from Source](https://elinux.org/Jetson/Installing_OpenCV)

### Install ROS Melodic
Following the instructions given in the [ROS Docs](https://wiki.ros.org/melodic/Installation/Ubuntu)
```
* sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
* sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
* sudo apt update
```
```
* sudo apt install ros-melodic-desktop-full
* echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
* source ~/.bashrc
```
```
* sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
* sudo apt install python-rosdep
* sudo rosdep init
* rosdep update
```

### Setup ROS Catkin-Workspace and Download needed Packages
Create folder structure for catkin workspace
```
mkdir -p ~/catkin_ws/src
```
#### Clone packages into src folder inside workspace
Due to the structure of ROS, functionalities are structured in packages. The following packages need to be installed to be able to use all functionalities of the system. If you only need to visualize data from CAN-bus you can skip everything related to the realsense SDK from Intel referring to the cameras. 

* [ros_canopen](https://github.com/ros-industrial/ros_canopen): Forward incoming and outgoing CAN Messages to and from ROS topics, interface between logic and CAN-hardware.
```
cd ~/catkin_ws/src/
git clone https://github.com/ros-industrial/ros_canopen.git
```
* [Realsense Wrapper for ROS](https://github.com/IntelRealSense/realsense-ros): Communication with Intel 3D cameras, we are using two Intel D455 cameras
```
cd ~/catkin_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
sudo apt-get install ros-melodic-ddynamic-reconfigure
```
* [bfft_CAN_msgs_to_ROS_topic](https://github.com/Black-Forest-Formula-Team/bfft_CAN_msgs_to_ROS_topic): Decode incoming CAN messages and publish them to corresponding topics
```
cd ~/catkin_ws/src/
git clone https://github.com/Black-Forest-Formula-Team/bfft_CAN_msgs_to_ROS_topic.git
```
* [bfft_rosdata_to_database](https://github.com/Black-Forest-Formula-Team/bfft_rosdata_to_database): Take data recorded in [ROSBAGS](https://wiki.ros.org/rosbag) (internal data format) and export it into CSV files (one per topic). Use CSV files for data visualization purpose
```
cd ~/catkin_ws/src/
git clone https://github.com/Black-Forest-Formula-Team/bfft_rosdata_to_database.git
```

For more input please refer to the [Catkin Docs](https://wiki.ros.org/catkin/workspaces)

### Build Process
Now we are able to build the workspace (if we have all libraries installed) with the packages downloaded above. 
```
catkin_make
```
If a library is missing make sure to install it via ```sudo apt-get install ros-melodic-libraryname``` if its a ROS library or via ```pip3 install libraryname``` if its a python3 lib.

Source setup file to be able to execute ros commands from every terminal
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
____________________
## Getting started
Our setup includes the Jetson AGX, two D455 cameras, one IMU from Genesys (ADMA Slim) as well as several CAN-Sensors and actors (for example two motors, inverters, wheelspeed sensors, BMS, ...) as can be seen in the image below.
<p align="center">
  <img src = "https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/blob/master/img/bfft_autonomous_driving_setup_20210311.png" width=1000>
</p>

### Intel Camera - Setup
Connect Intel D455 or similar Intel camera via USB3.0 (make sure that you use a 3.0 cable...). To check if ROS Realsense SDK works accordingly try out:
```roslaunch realsense2_camera demo_pointcloud.launch```
A pointcloud displayed in [Gazebo](https://wiki.ros.org/gazebo) should show up.

### CAN Bus of Jetson AGX Xavier - Setup
#### Software Setup
To be able to receive and send CAN bus data from a sensor to the AGX you need to setup and wire the hardware. Please follow the tutorial under [Enabling CAN on Nvidia Jetson Xavier Developer Kit](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9) and find more input in this [repo](https://github.com/hmxf/can_xavier). The important lines of the tutorials are mentioned below as step per step setup. Direct citations are marked with "".

"To make the above CAN controllers configuration automatically done at system startup, create a file named enable_CAN.sh in the root directory and make it executable:" (last line is modified/added to open vim directly)
```
touch /enable_CAN.sh
sudo chmod 755 /enable_CAN.sh
sudo vim /enable_CAN.sh
```
"To make the above CAN controllers configuration automatically done at system startup, create a file named enable_CAN.sh in the root directory and make it executable:" (changed the rate to 1Mbits, press ```i``` to be able to modify file and "Escape key" and then ```:wq``` to save and exit vim after copying the code below into the file)
```
#!/bin/bash
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 1000000 dbitrate 2000000 berr-reporting on fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 2000000 berr-reporting on fd on
sudo ip link set up can0
sudo ip link set up can1

exit 0
```
"If the file /etc/rc.local already exists on your Jetson Xavier, skip to the next step. If it does not exist, go ahead and create it by running the following commands in terminal:" (added last line to open vim directly)
```
printf '%s\n' '#!/bin/bash' 'exit 0' | sudo tee -a /etc/rc.local
sudo chmod +x /etc/rc.local
sudo vim /etc/rc.local
```
Another vim will open, press ```i``` to be able to modify file. "Add the following line to the/etc/rc.local file before the exit 0 line:" (and press "Escape key" and then ```:wq``` to save and exit vim after copying the code below into the file).
```
sh /enable_CAN.sh &
```
Now reboot the system and your CAN0 and CAN1 for the Jetson AGX are all set up and ready to be wired! Check if they are available by typing ifconfig, you should see the can0 and can1 listed with usb and eth:
```
ifconfig
```
#### Hardware Setup
Follow part 2 of the already mentioned [Tutorial](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9) to wire the AGX to the CAN Tranceiver. For this project we only use CAN0 and connect the CAN transceiver pins to the AGX pins according to the layout of the tutorial. Make sure to always turn the AGX off and unplug the power source as well as grounding yourself before wiring anything! 
* Pin 1 - V++
* Pin 29 - RX
* Pin 30 - Ground
* Pin 31 - TX

Be aware that you will need a 120 Ohm resistor on each side when working with CAN bus and longer cable length. For very short cables it might work without one.

### Connecting IMU and sensors via CAN bus
By using the DB9 connector of the Genesys IMU it is possible to connect the CAN-transceiver of the AGX with a self soldered DB9 connector. CAN signal is transmitted and received on CAN-High and CAN-Low channels. For more input on CAN please see [CAN Tutorial](https://www.csselectronics.com/screen/page/simple-intro-to-can-bus/language/en).

Assuming that the previous steps worked correctly when setting up the AGX as well as building the ROS packages in the catkin workspace using ```catkin_make``` it should now be possible to call the following roslaunch command to begin the listening to CAN0, decoding the CAN messages and writing them to topics.
```
roslaunch bfft_CAN_msgs_to_ROS_topic Start_Data_Collection.launch 
```
If you would like to see data comming in you can try one of the following as long as the IMU is attached:
```
rostopic echo /imu/imu_data
rostopic echo /imu/gps_data
```
It is possible to get a list of all available topics by typing ```rostopic list```.

### How to start recording of Data into ROSBAGS
TBD
                                                                ____________________
## Usage examples
### Convenience scripts for AGX ROS remote control via Ethernet/Wifi
* [Scripts to enable remote control of AGX](https://github.com/Black-Forest-Formula-Team/bfft_scripts): Useful if not monitor is available to connect AGX to, for example when racecar is standing on the race track. Remote control needed to start & stop autonomous system or to copy the recorded data from AGX to Windows PC.
```
cd
git clone https://github.com/Black-Forest-Formula-Team/bfft_scripts.git
```

### Start autonomous system from Windows laptop
```
sh ~/scripts/startROS.sh
```

### Stop autonomous system from Windows laptop
Kills all ROS processes including ROSBAG recodings
```
sh ~/scripts/stopROS.sh
```

### Convert ROSBAG to CSV file
```
sh ~/scripts/rosbagToCSV.sh
```

### Copy CSV files with CAN-Data to Windows Laptop
Make sure you are running these commands not on the AGX (with Ubuntu) but on a Laptop or PC with Windows. The purpose of this is to be able to copy the recorded and extracted CAN data now available in CSV files (one per topic) onto a Windows system and to display them using a tool of your choice. We are using Tableau for this. Automation of the copying and display process is currently in progress.

TBD

### Display Data in Tableau
TBD

____________________
## Features Datavisualization

A few of the things you can do with the data visualization plattform:

* View data of last test run
* ...

<p align="center">
  <img src = "http://XXXX.png" width=100>
</p>

________________________________
## Code Repository Conventions
### Python 
[PEP-8 style](http://wiki.ros.org/PyStyleGuide)
### ROS Python
[REP-8 style](https://www.ros.org/reps/rep-0008.html)

## ROS naming conventions
Thanks to [AMZ](https://github.com/AMZ-Driverless/fsd_skeleton) for the nice overview:
We use the naming conventions defined at http://wiki.ros.org/ROS/Patterns/Conventions
### Work packages
`work_package`, lowercase and `_` as separator, e.g. `lidar`.
### ROS packages
`workpackage_somename`, lowercase and `_` as separator, e.g. `lidar_trimmer`, as to make it clear what the package is used for.
### ROS nodes
`node_name`, lowercase and `_` as separator. Can be short.
### ROS topics
`topic_name`, lowercase and `_` as separator.
### ROS messages
`CamelCased.msg` for message filenames. Message types are always CamelCase, whereas message fields are lowercase and `_` as separator, e.g.
```
MyMessage.msg:
Header header
Float64 my_float
geometry_msgs/Point my_point
```

____________________
## Feedback

Feel free to send us feedback! If you wish to contribute, please take a quick look at the [guidelines](./CONTRIBUTING.md)!

If there's anything you'd like to chat about, please feel free to text us on one of our social media plattforms [Instagram](https://www.instagram.com/black_forest_formula/) [Facebook](https://www.facebook.com/blackforestformula/) [LinkedIN](https://linkedin.com/company/20527126)!


____________________
## Sponsors
Support this project by becoming a sponsor. Your logo will show up here with a link to your website. [[Become a sponsor](https://blackforestformula.hs-offenburg.de)]

____________________
## Acknowledgments
Thanks to ...

____________________
## Our Developers
Dev-Team Vehicle Control Unit & Autonomous Driving
* [Alex Sperka](https://github.com/AlexSperka) - Initial work
* Name2 - Initial work

____________________
## Release History
* 0.0.1
    * Initial setup, work in progress

____________________
## Meta
Distributed under the XYZ license. See ``LICENSE`` for more information.

____________________
## Contributing to one of our Repos
1. Fork it (<https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

