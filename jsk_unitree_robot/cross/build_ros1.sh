#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-arm64v8}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System
SOURCE_ROOT=${TARGET_MACHINE}_ws_system
MAKEFLAGS=${MAKEFLAGS:-'-j4'}

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

set -xeuf -o pipefail


# put ros1_dependencies.bash
mkdir -p ${HOST_INSTALL_ROOT}
cp ${PWD}/startup_scripts/ros1_dependencies_setup.bash ${HOST_INSTALL_ROOT}

# generate pr2eus package and install them within ${SOURCE_ROOT}
mkdir -p ${SOURCE_ROOT}/src
mkdir -p ${HOST_INSTALL_ROOT}/ros1_inst

if [ ${UPDATE_SOURCE_ROOT} -eq 1 ]; then
    vcs import --force --retry 3 --shallow ${SOURCE_ROOT}/src < repos/roseus_no_window.repos
    for dir in euslisp jskeus; do ls ${SOURCE_ROOT}/src/$dir/patches/; rsync -avz ${SOURCE_ROOT}/src/$dir/patches/ ${SOURCE_ROOT}/src/$dir; done
    sed -i s@:{version}@0.0.0@ ${SOURCE_ROOT}/src/euslisp/package.xml ${SOURCE_ROOT}/src/jskeus/package.xml
fi
wget https://patch-diff.githubusercontent.com/raw/PR2/pr2_mechanism/pull/346.diff -O ${SOURCE_ROOT}/pr2_mechanism-346.diff

EUSCOLLADA_DEPENDS="assimp_devel collada_parser collada_urdf resource_retriever"
ROSEUS_DEPENDS="actionlib_tutorials"
ROSEUS_SMACH_DEPENDS="smach_ros"
ROSEUS_MONGO_DEPENDS="mongodb_store_msgs"  # mongodb_store is ignored
JSK_ROBOT_STARTUP_DEPENDS="jsk_topic_tools pr2_mechanism_controllers gmapping jsk_recognition_msgs pointcloud_to_laserscan posedetection_msgs rosbridge_server roswww tf2_geometry_msgs jsk_tilt_laser"  # app_manager will be installed in User space, mongodb_store is ignored
JSK_ROBOT_UTILS="jsk_network_tools"
DIAGNOSTIC_AGGREGATOR="diagnostic_aggregator"  # jsk_XXX_startup usually depends on diagnostic_aggregator
PR2EUS="pr2eus"

docker run -it --rm \
  -u $(id -u $USER) \
  -e INSTALL_ROOT=${INSTALL_ROOT} \
  -e MAKEFLAGS=${MAKEFLAGS} \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/opt/jsk/${INSTALL_ROOT}/ros1_dependencies:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies_setup.bash:/opt/jsk/${INSTALL_ROOT}/ros1_dependencies_setup.bash:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_inst:/opt/jsk/${INSTALL_ROOT}/ros1_inst:rw \
  -v ${PWD}/${SOURCE_ROOT}:/home/user/${SOURCE_ROOT}:rw \
  ros1-unitree:${TARGET_MACHINE} \
  bash -c "\
    source /opt/jsk/System/ros1_dependencies_setup.bash && \
    source /opt/ros/melodic/setup.bash && \
    env && \
    set -xeuf -o pipefail && \
    rospack list && \
    cd ${SOURCE_ROOT} && \
    [ ${UPDATE_SOURCE_ROOT} -eq 0 ] || ROS_PACKAGE_PATH=src:\$ROS_PACKAGE_PATH rosinstall_generator ${EUSCOLLADA_DEPENDS} ${ROSEUS_DEPENDS} ${ROSEUS_MONGO_DEPENDS} ${ROSEUS_SMACH_DEPENDS} ${JSK_ROBOT_STARTUP_DEPENDS} ${DIAGNOSTIC_AGGREGATOR} ${PR2EUS} --verbose --deps --rosdistro melodic --exclude RPP --depend-type buildtool buildtool_export build run | tee unitree_ros1_system.repos && \
    [ ${UPDATE_SOURCE_ROOT} -eq 0 ] || PYTHONPATH= vcs import --force --retry 3 --shallow src < unitree_ros1_system.repos && \
    [ ! -e pr2_mechanism-346.diff ] || OUT=\"\$(patch -p1 --forward --directory src/pr2_mechanism < pr2_mechanism-346.diff | tee /dev/tty)\" || echo \"\${OUT}\" | grep \"Skipping patch\" -q || (echo \"\$OUT\" && false) && \
    catkin_make_isolated --install --install-space /opt/jsk/${INSTALL_ROOT}/ros1_inst -DCMAKE_BUILD_TYPE=Release \
        -DCATKIN_ENABLE_TESTING=FALSE \
        -DEUSLISP_WITHOUT_DISPLAY=TRUE -DDISABLE_DOCUMENTATION=1 \
    " 2>&1 | tee ${TARGET_MACHINE}_build_ros1.log

cp ${PWD}/startup_scripts/system_setup.bash ${HOST_INSTALL_ROOT}/
