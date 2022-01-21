# SR-DEV-BOBERT
Team Mission Control's Repository for UF CpE Senior Design 2022
- Luke Rouleau
- Xuanhao Shi

## Table of Contents
1. [Design Goals](#design-goals)
2. [Architecture](#architecture)
   - [Hardware](#hardware-overview)
   - [Software](#software-overview)
3. [Testing](#testing)
4. [Repository Contents](#repository-contents)
   - [ROS Melodic Implemenation](./ROS_Melodic_Implementation)
     - **Active:** This is the current path of development. 
     - ROS Melodic is natively supported by the Nvidia Jetson Hardware, making development significantly more intuitive than using a containerized approach.
   - [ROS2+Docker Approach](./ROS2_Docker_Implementation)
     - **Depreciated:** This was a design attempt from which we've since pivoted away. 
   
   
## Design Goals
### Top-Level Goal:
The goal for our team, *Mission Control*, is to successfully program a robot named “Bobert”, designed and built by the *UF 
IEEE Hardware Team*, to compete at *IEEE SoutheastCon*.

### What are the objectives specified by the IEEE Southeast Con Competition?
The objective of the competition is to autonomously clear an L-shaped track, which models a city corner, of debris (i.e., a marshmallow placed somewhere on the road) and to remove Mardi Gras-style bead necklaces from "trees" and deposit them into plastic cups or throw them into nets. Most of the track remains static, but the location of the "trees" are random. View the [official competiton document](./docs/2022_SoutheastCon_HardwareRules-Final.pdf).
<p align="center">
   <img src="./images/southeast_con_2022_competition_course.PNG" width="400">
</p>

### How does *Mission Control* plan to accomplish those goals?
Given the hardware provided, (a wheeled robotic platform, a Nvidia Jetson, Intel RealSense Depth and Tracking Cameras, and a servo-controlled 6 degree-of-freedom arm), we plan to connect the pieces of the autonomous robotic puzzle (perception, processing, actuation) with ROS Meloldic Middleware. What this really means is:
- [Install ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) on the Nvidia Jetson Nano
- Install [Intel's drivers](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md) to access the RealSense camera feeds inside of the /dev directory of Linux
- Install the RealSense Wrappers and Nodes into the ROS underlay (application-generic library code visible to ROS):
  - ```sudo apt-get install ros-$ROS_DISTRO-realsense2-camera```
- Create a ROS workspace in which a master node, camera nodes and topics, are created
  - Inside this repo, this workspace is the [./Ros_Melodic_Implementation/jetson_dev/cattkin_ws/](./Ros_Melodic_Implementation/jetson_dev/cattkin_ws/)
  - [Create unit tests](PUT LINK HERE) to verify the proper spawning of all nodes and datafeeds
- Employ SLAM, Mapping, and Navigation ROS packages for the competition environment.
- Use Move-It to perform inverse kinematics on the arm when the wheeled platform is in a correct location.
- **Wheel-Servo Drivers:** Compose a custom hardware interface that reads from a ROS topic to convert the planned path to wheeled motion.
- **Arm-Servo Driver:** Compose a custom hardware interface that reads from a ROS topic to control the 6DOF arm once the robot platform is near a "tree"
- Test Bobert inside of a replica course made by the IEEE Hardware Team

## Architecture
### Hardware Overview
1. Bobert's Mechanical Platform:
   - The form-factor is rectangular for ease and efficiency of mounting the rectangular internal components, the carriage is a tri-wheel design for maximal maneuverability  derived from a simple two-servo control scheme, and a top mounted arm for full range of motion above the robot body.
   - A 6DOF, servo-driven, [arm](https://www.amazon.com/diymore-Aluminium-Mechanical-Robotic-Arduino/dp/B01LW0LUPT/ref=asc_df_B01LW0LUPT/?tag=hyprod-20&linkCode=df0&hvadid=312031085430&hvpos=&hvnetw=g&hvrand=11766192266075412935&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9011703&hvtargid=pla-571106863858&psc=1)

2. Bobert's Electrical and Computational Hardware:
   - An [Intel RealSense T265 Tracking Camera](https://www.intelrealsense.com/tracking-camera-t265/)
   - An [Intel RealSense D435 Depth Camera](https://www.intelrealsense.com/depth-camera-d435/)
   - A [Nvidia Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) to perform image processing, running ROS, SLAM, path planning, inverse kinematics
   - A [Teensy Microcontroller](https://www.pjrc.com/store/teensy41.html) to drive the arm and wheel servos based on the ROS messages devlivered to it from the Jetson.
   
3. Hardware Component Relationship Diagram:
<p align="center">
   <img src="./images/component_relationship_diagram.PNG" width="600">
</p>
 

### Software Overview
1. Tools Used & Their Purpose
   - Base OS:
     - [JetPack](https://developer.nvidia.com/embedded/jetpack): Which is Ubuntu 18.04, with modifications made by Nvidia to fully utilize the GPU on the Jetson Nano
   - Perception & Planning:
     - **Intel RealSense SDK:** Exposes Open3D, TensorFlow, and OpenVINO packages, ROS Nodes, and install the hardware drivers for the T265 and D435 cameras 
     - [jetson-inference](https://github.com/dusty-nv/jetson-inference): Provides a Nvidia Deep-Learning API, which we will employ via transfer learning for object identification
   - Middleware: 
     - ROS Melodic: Not really and OS, but a "glue" technology that manages and handles communication between robotic elements
   - Visualization and Simulation:
     - Rviz: a visualization tool to view complex data (spacial data), check component models. Used to answer the question, "What does the robot see?"
     - Gazebo: a simulation tool for "what should the robot see?" and "what should the robot do after seeing?". This tool is mainly used when physical testing/experimentation is not feasible, but we mostly plan to experiment in person on a mock track.
   - Inverse Kinematics:
     - MoveIt!: a tool that, given a URDF file for an arm, performs inverse kinematics and path planning for the robotic arm
   - Development Environemnt: 
     - RoboWare: an IDE to set up the envrionment and organize all the source code (cfg files, include files, launch files, msg files, src files, CMakeLists, packages, etc.)
     
2. Relationship Diagram
   - In a PP somewhere
3. Interfaces
   - BB diagram on ipad and add it here

### Responsibilities
- Luke's Reponsibilites: 
  - Preparation of the Nvidia Jetson Environment
  - Creation the ROS environment
  - Connecting perception (RealSense Cameras) to ROS
- Xuanhao's Responsibilites:  
  - Link to his repo which includes his paper trail 


## Testing





# Repository Contents
## [Native ROS Melodic Implementation](./ROS_Melodic_Implementation)  
Based entirely off of [Nvidia's example](https://github.com/dusty-nv/jetbot_ros/tree/melodic)

### Goal:
Create a ROS-running autonomous robot powered by the Nvidia Jetson. Since the Jetson runs Ubuntu 18, ROS Melodic runs natively! Yay!
### Steps Taken [as of Alpha Build 1/21/22]:
- Install ROS Melodic (Desktop-Full install) onto the Nvidia Jetson 
- Install Gazebo for ROS Melodic
- Install the RealSense Library & Wrappers for ROS Melodic
- Create a [Roboware](http://wiki.ros.org/IDEs#RoboWare_Studio) IDE compatible [workspace](./ROS_Melodic_Implementation/roboware_ros_ws/)
- Built [jetson-inference](https://github.com/dusty-nv/jetson-inference), Nvidia's DNN repository for the Jetson Platform, onto the Jetson.
- Built the [ROS Melodic Nodes](https://github.com/dusty-nv/ros_deep_learning) to interact with **jetson-inference** from our cameras.
- [Connect the RealSense D435 and T265 cameras to the ROS environment, spawn their nodes.](./ROS_Melodic_Implementation/jetson_dev/images/d435_and_t265_rosgraph.png)
- Connect the camera input to Rviz (click image for video):

 [![Connect the camera input to Rviz](https://img.youtube.com/vi/0Snpj9SjjzQ/0.jpg)](https://www.youtube.com/watch?v=0Snpj9SjjzQ)



## [DEPRECIATED!: ROS2-Docker Implementation](./ROS2_Docker_Implementation)
Based entirely off of [Nvidia's example](https://github.com/dusty-nv/jetbot_ros)

### Goal:
Create a ROS2-running robot powered by the Nvidia Jetson. Since the Jetson runs Ubuntu 18, use docker containers to accomplish this.
### Steps Taken [as of Alpha Build 1/21/22]:
- Create a [Dockerfile](./ROS2_Docker_Implementation/Dockerfile) to build a custom container to our specs.
- Test the docker container, try to start a Gazebo World. **Failure, detailed below.**

### Issues Present [as of Alpha Build 1/21/22]:
- Though we can successfully build a ROS2 Foxy container with Gazebo and Python 3 Bindings for Gazebo, Gazebo ALWAYS hangs on launch, like seen [here](./ROS2_Docker_Implementation/imgs/gazebo_hang.jpeg).
- We cannot escape the feeling that we are trying to stick a round peg into a square hole with trying to run ROS2 on the Jetson. 
  - **It might be time to fallback to ROS. - Luke, 1/17/22**
