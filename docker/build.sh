#!/bin/bash

TARGET="$1"

PLATFORM="$(uname -m)"

BASE_NAME="ros_custom-$TARGET-$PLATFORM"

# Build the image
docker buildx build ./docker \
	--file ./docker/Dockerfile.$TARGET \
	--build-arg USERNAME=admin \
	--build-arg USER_UID=1000 \
	--build-arg USER_GID=1000 \
	--build-arg SSH_PRV_KEY="$(cat ~/.ssh/id_rsa)" \
	--build-arg SSH_PUB_KEY="$(cat ~/.ssh/id_rsa.pub)" \
	--platform $PLATFORM \
	--tag $BASE_NAME
