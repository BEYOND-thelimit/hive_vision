#!/bin/bash

# Sample script to run a command in a Docker container

# Define Docker volumes and environment variables
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
--volume="/dev/video0:/dev/video0" \
--volume="/dev/video2:/dev/video2" \
"
DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
"
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}


# $1: Docker image name
# $2: Docker container name
docker run -it --rm --net=host --ipc=host --privileged --gpus all ${DOCKER_ARGS} --name "$2" "$1" /bin/bash