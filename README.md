# SR-DEV-BOBERT
Team Mission Control's Repository for UF CpE Senior Design 2022
- Luke Rouleau
- Xuanhao Shi

## [ROS2-Docker Implementation](./ROS2_Docker_Implementation)
### Goal:
Create a ROS2-running robot powered by the Nvidia Jetson. Since the Jetson runs Ubuntu 18, use docker containers to accomplish this.
### Steps Taken [as of Alpha Build 1/21/22]:
- Create a [Dockerfile]() to build a custom container to our specs.
- Test the docker container, try to start a Gazebo World. 

### Issues Present [as of Alpha Build 1/21/22]:
- Though we can successfully build a ROS2 Foxy container with Gazebo and Python 3 Bindings for Gazebo, Gazebo always hangs on launch, like seen [here]().
- We cannot escape the feeling that we are trying to stick a round peg into a square hole with trying to run ROS2 on the Jetson. 
  - **It might be time to fallback to ROS. - Luke, 1/17/22**


## [Native ROS Melodic Implementation](./ROS_Melodic_Implementation)  
TBF
