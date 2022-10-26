#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-arm64v8}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System
SOURCE_ROOT=${TARGET_MACHINE}_ws_ros1_dependencies_sources
MAKEFLAGS=${MAKEFLAGS:-'-j4'}

if [ -e "${SOURCE_ROOT}" ]; then
    echo "WARNING: Source directory is found ${SOURCE_ROOT}" 1>&2
    read -p "WARNING: Are you sure to continue [y/N] ? " -n 1 -r
    echo    # (optional) move to a new line
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "WARNING: Exitting.. please remove ${SOURCE_ROOT} and start again"
        exit 1
    fi
fi

set -xeuf -o pipefail

# install development libraries requried for ros1 compile
mkdir -p ${HOST_INSTALL_ROOT}/ros1_dependencies
mkdir -p ${SOURCE_ROOT}/src
cp repos/ros1_dependencies.repos ${SOURCE_ROOT}/

mkdir -p ${HOST_INSTALL_ROOT}/Python
cp repos/go1_requirements.txt ${SOURCE_ROOT}/go1_requirements.txt

docker run -it --rm \
  -u $(id -u $USER) \
  -e INSTALL_ROOT=${INSTALL_ROOT} \
  -e MAKEFLAGS=${MAKEFLAGS} \
  -v ${PWD}/ros1_dependencies_build_scripts:/home/user/ros1_dependencies_build_scripts:ro \
  -v ${PWD}/${SOURCE_ROOT}:/home/user/ros1_dependencies_sources:rw \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/opt/jsk//${INSTALL_ROOT}/ros1_dependencies:rw \
  -v ${HOST_INSTALL_ROOT}/Python:/opt/jsk/${INSTALL_ROOT}/Python:rw \
  ros1-unitree:${TARGET_MACHINE} \
  bash -c "\
    set -xeuf -o pipefail && \
    cd /home/user/ros1_dependencies_sources && \
    vcs import --skip-existing --retry 3 --shallow src < ros1_dependencies.repos && \
    for script_file in \$(ls /home/user/ros1_dependencies_build_scripts/|sort); do
      /home/user/ros1_dependencies_build_scripts/\$script_file || exit 1;
    done && \
    pip install -U --user pip && \
    export PYTHONPATH=\"/opt/jsk/System/ros1_dependencies/lib/python2.7/site-packages\" && \
    export PKG_CONFIG_PATH=\"/opt/jsk/${INSTALL_ROOT}/ros1_dependencies/lib/pkgconfig\" && \
    ~/.local/bin/pip install --prefix=/opt/jsk/${INSTALL_ROOT}/Python -r /home/user/ros1_dependencies_sources/go1_requirements.txt \
    " 2>&1 | tee ${TARGET_MACHINE}_build_ros1_dependencies.log
