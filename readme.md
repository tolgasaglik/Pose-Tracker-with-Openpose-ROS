# Gesture Detector with Openpose Software on ROS Kinetic

This repository aims to recognize gestures made by users. Recognized gestures can lead the robot to take certain actions and/or repeat user's movement.

## Table of Contents
* [Dependencies](#dependencies)
* [Compatible Platforms](#compatible-platforms)
* [Hardware Requirements](#hardware-requirements)
* [Installation](#installation)
* [License](#license)

## Dependencies

1. ROS Kinetic
2. Openpose Software
3. Librealsense SDK 2.10.0+ (included inside the repo)


## Compatible Platforms

The library is written in standards-conforming Python. It is developed and tested on the following platforms:

1. Ubuntu 16.04
2. ROS Indigo / Kinetic
3. CUDA 8.0
4. cuDNN 5.1
5. OpenCV 3.2


## Hardware Requirements
Project uses following hardware:

1. Intel Core-i7 6700HQ
2. Nvidia Mobile Gtx 960
3. RealSense SR300 (Firmware 3.10.10.0 and up)
4. USB 3.0 Type-A port for camera connection


## Installation

As the ROS will be main environment it needs to be installed first. You may switch between camera and openpose installation. However completing step 3 before 4 is vital to prevent kernel errors.

1. Install ROS Kinetic: http://wiki.ros.org/kinetic/Installation/Ubuntu
2. Install Openpose and OpenposeROS node following instructions from: https://github.com/firephinx/openpose_ros
3. Install Realsense Camera Prerequisities: http://wiki.ros.org/librealsense#Installation_Prerequisites
4. Install Realsense Camera ROS Node from source: http://wiki.ros.org/realsense_camera/Tutorials/Building_librealsense_from_Sources


## License

Copyright 2018 University of Luxembourg

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this project except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
