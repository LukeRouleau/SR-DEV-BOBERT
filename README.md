# SR-DEV-BOBERT
Team Mission Control's Repository for UF CpE Senior Design 2022
- Luke Rouleau
- Xuanhao Shi

## [ROS2-Docker Implementation](./ROS2_Docker_Implementation)
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


## [Native ROS Melodic Implementation](./ROS_Melodic_Implementation)  
Based entirely off of [Nvidia's example](https://github.com/dusty-nv/jetbot_ros/tree/melodic)

### Goal:
Create a ROS-running autonomous robot powered by the Nvidia Jetson. Since the Jetson runs Ubuntu 18, ROS Melodic runs natively! Yay!
### Steps Taken [as of Alpha Build 1/21/22]:
- Install ROS Melodic onto the Nvidia Jetson 
- Install Gazebo for ROS Melodic
- Install the RealSense Library & Wrappers for ROS Melodic
- Create a [Roboware](http://wiki.ros.org/IDEs#RoboWare_Studio) IDE compatible [workspace](./ROS_Melodic_Implementation/ros_ws/)
