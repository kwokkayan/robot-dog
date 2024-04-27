#!/bin/bash

TARGET="$1"

PLATFORM="$(uname -m)"

BASE_NAME="ros_custom-$TARGET-$PLATFORM"

if [[ $TARGET == "bridge" ]]; then
	BUILD_ARGS+=("--build-arg USERNAME=admin")
	BUILD_ARGS+=("--build-arg USER_UID=1000")
	BUILD_ARGS+=("--build-arg USER_GID=1000")
fi

if [[ $TARGET == "noetic" ]]; then
	BUILD_ARGS+=("--build-arg USERNAME=admin")
	BUILD_ARGS+=("--build-arg USER_UID=1000")
	BUILD_ARGS+=("--build-arg USER_GID=1000")
	BUILD_ARGS+=("--build-context src=../workspaces/ros/noetic/")
fi

if [[ $TARGET == "humble" ]]; then
	BUILD_ARGS+=("--build-arg USERNAME=admin")
	BUILD_ARGS+=("--build-arg USER_UID=1000")
	BUILD_ARGS+=("--build-arg USER_GID=1000")
	BUILD_ARGS+=("--build-context src=../workspaces/ros/humble/")
fi

if [[ $TARGET == "newviz" ]]; then
	BUILD_ARGS+=("--build-context src=../workspaces/newviz/")
fi

# Build the image
sudo docker buildx build ./context \
	${BUILD_ARGS[@]} \
	--file ./context/Dockerfile.$TARGET \
	--platform $PLATFORM \
	--tag $BASE_NAME
