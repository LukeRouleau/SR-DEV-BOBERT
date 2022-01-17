#!/usr/bin/env bash
#
# This script builds the bobert_custom docker container from source.
# It should be run from the root dir of the bobert_custom project:
#
#     $ cd /path/to/your/bobert_custom
#     $ docker/build.sh
#
# Also you should set your docker default-runtime to nvidia:
#     https://github.com/dusty-nv/jetson-containers#docker-default-runtime
#
ROS_DISTRO=${1:-"foxy"}
BASE_IMAGE=$2

# break on errors
set -e

# find L4T_VERSION
source docker/tag.sh

if [ -z $BASE_IMAGE ]; then
	BASE_IMAGE="dustynv/ros:$ROS_DISTRO-pytorch-l4t-$TAG"
fi


build_container()
{
	local container_image=$1
	local dockerfile=$2
	
	echo "building $container_image"
	echo "BASE_IMAGE=$BASE_IMAGE"

	sudo docker build -t $container_image -f $dockerfile \
			--build-arg BASE_IMAGE=$BASE_IMAGE \
			.
}

build_container "bobert_custom:$ROS_DISTRO-$TAG" "Dockerfile"
