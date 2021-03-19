# bfft_formula-student_driverless

<p align="center">
  <a href="https://blackforestformula.hs-offenburg.de/">
    <img alt="BFFT_Logo" title="BFFT" src="https://scontent-frt3-1.xx.fbcdn.net/v/t1.0-9/69419451_117866062911797_4569414645357477888_o.jpg?_nc_cat=107&ccb=1-3&_nc_sid=973b4a&_nc_ohc=b5rqMomf8_AAX8x_CMD&_nc_ht=scontent-frt3-1.xx&oh=7ab30784f93fdf5ad846196156f856e6&oe=606D20C4" width="1000">
  </a>
</p>

# Black Forest Formula Team - Formula Student Driverless 2021
<!-- Template Readme:https://github.com/gitpoint/git-point#readme -->

This repository lays the foundation for the future developments of autonomous features of the Black Forest Formula Team located in Offenburg. You can find an overview to get started in this ReadMe, for more information we suggest to refer to the [Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki) you can find in this Repo. This repository as well as our subrepositories are created and maintained by the [Black Forest Formula Team](https://blackforestformula.hs-offenburg.de/) at [University of Applied Sciences Offenburg](https://www.hs-offenburg.de/). 
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
- [Feedback](#feedback)
- [Our Developers](#our-developers)
- [Release History](#release-history)
- [Meta](#meta)
- [Contributing to one of our Repos](#contributing-to-one-of-our-repos)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->
____________________
## Introduction
This setup and code implementation has three main purposes:
1. Visualize Data of our first electric racecar when doing testdrives and real races as close to the run as possible (best of course would be real-time)
2. Record all relevant incoming data while doing a test run for simulation purposes later on (be it for the controll system, temp simulation, object detection or others)
3. Build the foundation to push our car to one day drive with autonomous features

Therefore, we decided to use ROS (and soon ROS2) for implementing the given goals as stated above. Right now we are almost ready to fulfill goal Nr. 1 as well as goal Nr. 2. Until goal Nr. 3 there is still quite a way to got. Our progress of the software setup and how it plays together with our hardware can be seen in the image below.

<p align="center">
  <img src = "https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/blob/main/img/ROS_Architecture_20210311.svg" width=1000>
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

### Additional and mandatory Libraries and Tools
Please visit [this](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/01-Installation-Libraries-for-Workspace-(Python-and-ROS)) Wiki Page to install all tools and libraries you will need for this system to run.

### Install ROS Melodic
To get the system running we first have to install ROS1 melodic (and in the future probably ROS2). The needed steps are mentioned [here in the Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/01-Installation-ROS-1-Melodic). If you already installed ROS you can skip this step.

### Setup ROS Catkin-Workspace and Download needed Packages
Create folder structure for catkin workspace. If you already have one go ahead, you might need to adjust the folder path accordingly. 
```
mkdir -p ~/catkin_ws/src
```
#### Clone packages into src folder inside workspace
Due to the structure of ROS, functionalities are structured in packages. The following packages need to be installed to be able to use all functionalities of the system. If you only need to visualize data from CAN-bus you can skip everything related to the realsense SDK from Intel referring to the cameras. 

More detail on the setup process can be found in the [Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/02-Setup-Catkin-Workspace)

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
* [bfft_can_bus_msgs_to_ros_topic](https://github.com/Black-Forest-Formula-Team/bfft_can_bus_msgs_to_ros_topic): Decode incoming CAN messages and publish them to corresponding topics
```
cd ~/catkin_ws/src/
git clone https://github.com/Black-Forest-Formula-Team/bfft_can_bus_msgs_to_ros_topic.git
```
* [bfft_rosbag_data_conversion](https://github.com/Black-Forest-Formula-Team/bfft_rosbag_data_conversion): Take data recorded in [ROSBAGS](https://wiki.ros.org/rosbag) (internal data format) and export it into CSV files (one per topic). Use CSV files for data visualization purpose
```
cd ~/catkin_ws/src/
git clone https://github.com/Black-Forest-Formula-Team/bfft_rosbag_data_conversion.git
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

More detail in the [Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/03-Build-Process-Catkin-Workspace).

____________________
## Getting started
Our setup includes the Jetson AGX, two D455 cameras, one IMU from Genesys (ADMA Slim) as well as several CAN-Sensors and actors (for example two motors, inverters, wheelspeed sensors, BMS, ...) as can be seen in the image below.
<p align="center">
  <img src = "https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/blob/main/img/bfft_autonomous_driving_setup_20210311.png" width=1000>
</p>

### Intel Camera - Setup
Connect Intel D455 or similar Intel camera via USB3.0 (make sure that you use a 3.0 cable...). To check if ROS Realsense SDK works accordingly try out:
```roslaunch realsense2_camera demo_pointcloud.launch```
A pointcloud displayed in [Gazebo](https://wiki.ros.org/gazebo) should show up.

[Here](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/02-Realsense-SDK-on-AGX) you can find a little guide with more input on the camera setup.

### CAN Bus of Jetson AGX Xavier - Setup
To be able to receive and send CAN bus data from a sensor to the AGX you need to setup and wire the hardware. This is described in our Wiki [here](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/02-Setup-CAN-Communication-on-NVIDIA-Jetson-AGX-Xavier).


### Connecting IMU and sensors via CAN bus
Our [Wiki](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/02-Setup-ADMA-Slim-IMU-from-Genesys-using-CAN) guide for this section can be found here.

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
Start CAN connection, read in messages and transform them into ROS topics, save everything as ROSBAGs 
```
sh ~/scripts/startROS.sh
```

### Stop autonomous system from Windows laptop
Kills all ROS processes including ROSBAG recodings
```
sh ~/scripts/stopROS.sh
```

### Convert ROSBAG to CSV file
Convert the latest (or a specified) ROSBAG into CSV files (on per topic) to be able to display them in Tableau or other visualization apps
```
sh ~/scripts/rosbagToCSV.sh
```

![ROSBAG to CSV](demo/convert-ROSBAG-to-CSV-script.gif)

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
For our coding conventions please visit the wiki page [ROS & Python Conventions](https://github.com/Black-Forest-Formula-Team/bfft_formula-student_driverless/wiki/00-Coding-Conventions)!

____________________
## Feedback

Feel free to send us feedback! 

If there's anything you'd like to chat about, please feel free to text us on one of our social media plattforms: 
* [Instagram](https://www.instagram.com/black_forest_formula/) 
* [Facebook](https://www.facebook.com/blackforestformula/) 
* [LinkedIN](https://linkedin.com/company/20527126)!

Support this project by becoming a sponsor. Your logo will show up on our website with a link to your website. [[Become a sponsor](https://blackforestformula.hs-offenburg.de)]

____________________
## Our Developers
Dev-Team Vehicle Control Unit & Autonomous Driving in alphabetical order
* [Alex](https://github.com/AlexSperka) - Initial work
* [Benedikt](https://github.com/newtop95) - Initial work
* [Steffi](https://github.com/steffistae) - Initial work
* [Tizian](https://github.com/tdagner) - Initial work

____________________
## Release History
* 0.0.1
    * Initial setup, work in progress

____________________
## Meta
Distributed under the MIT license. See ``LICENSE.md`` for more information.

____________________
## Contributing to one of our Repos
1. Fork it (<https://github.com/Black-Forest-Formula-Team/bfft_can_bus_msgs_to_ros_topic/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

____________________


<p align="left">
  <a href="https://www.hs-offenburg.de/">
      <img alt="HSO_Logo" title="HSO_Logo" src="https://static.onthehub.com/production/attachments/15/66edb074-5e09-e211-bd05-f04da23e67f6/7978f7db-e206-4cd7-b7b2-6d9696e98885.png" width="1000">
  </a>
</p>
