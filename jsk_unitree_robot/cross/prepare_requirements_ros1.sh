#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-arm64v8}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"System
INSTALL_ROOT=System

set -xeuf -o pipefail

# setup ros1-unitree docker, which installed package also installed in the robot
if ! docker buildx > /dev/null; then
    DOCKER_BUILDKIT=1 docker build --platform=local -o . "https://github.com/docker/buildx.git"
fi
if [ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ] &&  [ ! "$(docker images -q multiarch/qemu-user-static)" ]; then
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
fi

docker buildx build $@ --progress plain -t ros1-unitree:${TARGET_MACHINE} --build-arg TARGET_MACHINE=${TARGET_MACHINE} --build-arg UID=$(id -u $USER) -f docker/Dockerfile_ros1 docker/ 2>&1 | tee ${TARGET_MACHINE}_prepare_requirements_ros1.log
