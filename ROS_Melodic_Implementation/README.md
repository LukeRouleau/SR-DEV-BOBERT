# SR-DEV-BOBERT ROS Melodic Implementation
Create a ROS-running autonomous robot powered by the Nvidia Jetson. Since the Jetson runs Ubuntu 18, ROS Melodic runs natively! Yay!


## State of Affairs

### Steps Taken [as of Alpha Build 1/21/22]:
- Install ROS Melodic onto the Nvidia Jetson 
  - Based entirely off of [Nvidia's example](https://github.com/dusty-nv/jetbot_ros/tree/melodic) 
- Install Gazebo for ROS Melodic
- Install the RealSense Library & Wrappers for ROS Melodic
- Create a [Roboware](http://wiki.ros.org/IDEs#RoboWare_Studio) IDE compatible [workspace](./ROS_Melodic_Implementation/ros_ws/)
- Built [jetson-inference](https://github.com/dusty-nv/jetson-inference), Nvidia's DNN repository for the Jetson Platform, onto the Jetson.
- Built the [ROS Melodic Nodes](https://github.com/dusty-nv/ros_deep_learning) to interact with **jetson-inference** from our cameras.

### What is RoboWare Studio?
RoboWare Studio is an IDE, based on Visual Studio, for ROS development. The ROS [workspace](./ros_ws/) we've created has been formatted to be compatible with the Roboware IDE:
1. Download and install [RoboWare Studio](http://www.roboware.me/) 
2. Open RoboWare Studio and select [ros_ws](./ros_ws/) as your workspace.
3. Click compile on the Roboware IDE
4. Develop away!
