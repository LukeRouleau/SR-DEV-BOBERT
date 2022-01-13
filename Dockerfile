# Author: Luke Rouleau
# Date: 1/13/2022
# Dockerfile for running ROS2 Conatiner on the Nvidia Jetson Nano For the IEEE Hardware Design Team
# robot named Bobert

# Based off the Dockerfile written by Nvidia viewable here:
#  https://github.com/dusty-nv/jetbot_ros/blob/master/Dockerfile

# Build this Dockerfile by running the following commands:
#
#     $ cd /path/to/your/jetbot_ros
#     $ docker/build.sh
#
# Also you should set your docker default-runtime to nvidia:
#     https://github.com/dusty-nv/jetson-containers#docker-default-runtime
#

ARG BASE_IMAGE=dustynv/ros:foxy-pytorch-l4t-r32.5.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"] 
ENV SHELL /bin/bash

ENV DEBIAN_FRONTEND=noninteractive
ARG MAKEFLAGS=-j$(nproc)
ENV LANG=en_US.UTF-8 
ENV PYTHONIOENCODING=utf-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

WORKDIR /tmp


#
# install gazebo & utilities
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		  nano \
		  xterm \
		  lxterminal \
		  blender \
		  libgazebo9-dev \
		  gazebo9 \
		  gazebo9-common \
		  gazebo9-plugin-base \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN git clone https://github.com/dusty-nv/py3gazebo /opt/py3gazebo && \
    pip3 install protobuf>=2.6 --verbose && \
    pip3 install trollius --verbose && \
    pip3 install pynput --verbose

ENV PYTHONPATH=/opt/py3gazebo
   
   
#
# Gazebo plugins for ROS
#
RUN source ${ROS_ROOT}/install/setup.bash && \
    export ROS_PACKAGE_PATH=${AMENT_PREFIX_PATH} && \
    cd ${ROS_ROOT} && \
    mkdir -p src/gazebo && \
    rosinstall_generator --deps --exclude RPP --rosdistro ${ROS_DISTRO} \
          gazebo_ros_pkgs \
	> ros2.${ROS_DISTRO}.gazebo.rosinstall && \
    cat ros2.${ROS_DISTRO}.gazebo.rosinstall && \
    vcs import src/gazebo < ros2.${ROS_DISTRO}.gazebo.rosinstall && \
    apt-get update && \
    rosdep install -y \
       --ignore-src \
       --from-paths src/gazebo \
	  --rosdistro ${ROS_DISTRO} \
	  --skip-keys "gazebo11 libgazebo11-dev libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv" && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    colcon build --merge-install --base-paths src/gazebo \
    && rm -rf ${ROS_ROOT}/src \
    && rm -rf ${ROS_ROOT}/logs \
    && rm -rf ${ROS_ROOT}/build 
    

#
# JetBot hw controllers
#
#RUN pip3 install Adafruit-MotorHAT Adafruit-SSD1306 --verbose


#
# environment setup
#   
ENV WORKSPACE_ROOT=/workspace
ENV JETBOT_ROOT=${WORKSPACE_ROOT}/src/jetbot_ros
ARG ROS_ENVIRONMENT=${ROS_ROOT}/install/setup.bash

ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:/root/.gazebo/models:${JETBOT_ROOT}/gazebo/models
ENV GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${JETBOT_ROOT}/gazebo/plugins/build/:/usr/local/lib/
ENV GAZEBO_MASTER_URI=http://localhost:11346

# setup workspace
WORKDIR ${WORKSPACE_ROOT}
RUN mkdir -p ${WORKSPACE_ROOT}/src

COPY scripts/setup_workspace.sh ${WORKSPACE_ROOT}/setup_workspace.sh
ENV PYTHONPATH="${JETBOT_ROOT}:${PYTHONPATH}"

    
#
# ros_deep_learning package
#
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT}/src && \
    git clone https://github.com/dusty-nv/ros_deep_learning && \
    cd ../ && \
    colcon build --symlink-install --event-handlers console_direct+


#
# build project
#
COPY jetbot_ros ${JETBOT_ROOT}/jetbot_ros
COPY launch ${JETBOT_ROOT}/launch
COPY gazebo ${JETBOT_ROOT}/gazebo
COPY resource ${JETBOT_ROOT}/resource

COPY package.xml ${JETBOT_ROOT}
COPY setup.py ${JETBOT_ROOT}
COPY setup.cfg ${JETBOT_ROOT}

RUN cd ${JETBOT_ROOT}/gazebo/plugins/ && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make -j$(nproc) && \
    make install
    
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT} && \
    colcon build --symlink-install --event-handlers console_direct+


#
# setup entrypoint
#
COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh

RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/install\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh

RUN echo 'source ${ROS_ROOT}/install/setup.bash' >> /root/.bashrc && \
    echo 'source ${WORKSPACE_ROOT}/install/local_setup.bash' >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]