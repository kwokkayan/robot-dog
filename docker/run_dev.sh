#!/bin/bash

podman run --rm -it \
	--privileged \
	--cap-add ALL \
	--network host \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	ros:humble-ros-base \
	/bin/bash -c "apt update && apt install -y ros-humble-rviz2 && /bin/bash"
