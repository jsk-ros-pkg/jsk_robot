#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-arm64v8}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System
SOURCE_ROOT=${TARGET_MACHINE}_User

TARGET_ROBOT="${TARGET_ROBOT:-unitree}"

set -euf -o pipefail

UPDATE_SOURCE_ROOT=1  # TRUE
if [ -e "${SOURCE_ROOT}" ]; then
    echo "WARNING: Source directory is found ${SOURCE_ROOT}" 1>&2
    echo "WARNING: It might contain old data. We recommend to remove ${SOURCE_ROOT} and start again" 1>&2
    read -p "WARNING: Are you sure to continue [y/N] ? " -n 1 -r
    echo    # (optional) move to a new line
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "WARNING: Exitting.."
        exit 1
    fi
    UPDATE_SOURCE_ROOT=0  # FALSE
fi

set -x
# copy jsk_robot direcotry to jsk_catkin_ws/src
mkdir -p ${SOURCE_ROOT}/src/jsk_robot
rsync -avzh --delete --exclude 'jsk_unitree_robot/cross*' --exclude "go1.l" --exclude "go1-simple.l" ../../../jsk_robot ${SOURCE_ROOT}/src/

# ingore non-related packages
for dir in $(find ${SOURCE_ROOT}/src/jsk_robot -maxdepth 1 -mindepth 1 -type d); do
    if [[ ! $dir =~ "jsk_${TARGET_ROBOT}_robot"|jsk_robot_common ]]; then
        touch $dir/CATKIN_IGNORE
    fi
done

# add unitree repos
[ ${UPDATE_SOURCE_ROOT} -eq 0 ] || vcs import ${SOURCE_ROOT}/src < repos/unitree.repos

# check if /proc/sys/fs/binfmt_misc/qemu-* is updated
# See https://github.com/k-okada/jsk_robot/issues/61
docker run -it --rm ros1-unitree:${TARGET_MACHINE} bash -c 'exit' ||  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# run on docker
docker run -it --rm \
  -u $(id -u $USER) \
  -e INSTALL_ROOT=${INSTALL_ROOT} \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/opt/jsk/${INSTALL_ROOT}/ros1_dependencies:ro \
  -v ${HOST_INSTALL_ROOT}/Python:/opt/jsk/${INSTALL_ROOT}/Python:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_inst:/opt/jsk/${INSTALL_ROOT}/ros1_inst:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies_setup.bash:/opt/jsk/${INSTALL_ROOT}/ros1_dependencies_setup.bash:ro \
  -v ${HOST_INSTALL_ROOT}/system_setup.bash:/opt/jsk/${INSTALL_ROOT}/system_setup.bash:ro \
  -v ${HOST_INSTALL_ROOT}/sitecustomize.py:/usr/lib/python2.7/sitecustomize.py:ro \
  -v ${HOST_INSTALL_ROOT}/sitecustomize.py:/usr/lib/python3.6/sitecustomize.py:ro \
  -v ${PWD}/${SOURCE_ROOT}:/opt/jsk/User:rw \
  -v ${PWD}/rosinstall_generator_unreleased.py:/home/user/rosinstall_generator_unreleased.py:ro \
  ros1-unitree:${TARGET_MACHINE} \
  bash -c "\
    source /opt/jsk/System/system_setup.bash && \
    env && \
    set -xeuf -o pipefail && \
    cd /opt/jsk/User && \
    [ ${UPDATE_SOURCE_ROOT} -eq 0 ] || ROS_PACKAGE_PATH=src:\${ROS_PACKAGE_PATH} /home/user/rosinstall_generator_unreleased.py jsk_${TARGET_ROBOT}_startup ${TARGET_ROBOT}eus --rosdistro melodic --exclude RPP --exclude mongodb_store | tee user.repos && \
    [ ${UPDATE_SOURCE_ROOT} -eq 0 -o -z \"\$(cat user.repos)\" ] || vcs import src < user.repos && \
    catkin build jsk_${TARGET_ROBOT}_startup ${TARGET_ROBOT}eus -s -vi \
        --cmake-args -DCATKIN_ENABLE_TESTING=FALSE \
    " 2>&1 | tee ${TARGET_MACHINE}_build_user.log
cp ${PWD}/startup_scripts/user_setup.bash ${SOURCE_ROOT}/
