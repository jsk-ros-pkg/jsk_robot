#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-i386}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System
SOURCE_ROOT=${TARGET_MACHINE}_User

PYTHON2_VERSION=2.7.17

TARGET_ROBOT="${TARGET_ROBOT:-pepper}"

ARGS="${@:-build jsk_${TARGET_ROBOT}_startup ${TARGET_ROBOT}eus}"

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
rsync -avzh --delete --exclude 'jsk_naoqi_robot/cross*' --exclude "nao.l" --exclude "pepper.l" ../../../jsk_robot ${SOURCE_ROOT}/src/

# ingore non-related packages
for dir in $(find ${SOURCE_ROOT}/src/jsk_robot -maxdepth 1 -mindepth 1 -type d); do
    if [[ ! $dir =~ jsk_naoqi_robot|jsk_robot_common ]]; then
        [ -e $dir/CATKIN_IGNORE ] || touch $dir/CATKIN_IGNORE
    fi
done

# add unitree repos
[ ${UPDATE_SOURCE_ROOT} -eq 0 ] || vcs import ${SOURCE_ROOT}/src < repos/pepper.repos

## UPDATE_SOURCE_ROOT=1  # TRUE

# run on docker
docker run -it --rm \
  -u $(id -u $USER) \
  -e ALDE_CTC_CROSS=/home/nao/ctc \
  -v ${ALDE_CTC_CROSS}:/home/nao/ctc:ro \
  -v ${PWD}/ctc-cmake-toolchain.cmake:/home/nao/ctc-cmake-toolchain.cmake:ro \
  -e INSTALL_ROOT=${INSTALL_ROOT} \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/home/nao/${INSTALL_ROOT}/ros1_dependencies:ro \
  -v ${HOST_INSTALL_ROOT}/Python-${PYTHON2_VERSION}:/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_inst:/home/nao/${INSTALL_ROOT}/ros1_inst:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies_setup.bash:/home/nao/${INSTALL_ROOT}/ros1_dependencies_setup.bash:ro \
  -v ${HOST_INSTALL_ROOT}/system_setup.bash:/home/nao/${INSTALL_ROOT}/system_setup.bash:ro \
  -v ${PWD}/${SOURCE_ROOT}:/home/nao/User:rw \
  -v ${PWD}/rosinstall_generator_unreleased.py:/home/user/rosinstall_generator_unreleased.py:ro \
  ros1-pepper:${TARGET_MACHINE} \
  bash -c "\
    source /home/nao/System/system_setup.bash && \
    env && \
    set -xeuf -o pipefail && \
    cd /home/nao/User && \
    [ ${UPDATE_SOURCE_ROOT} -eq 0 ] || ROS_PACKAGE_PATH=src:\${ROS_PACKAGE_PATH} /home/user/rosinstall_generator_unreleased.py jsk_${TARGET_ROBOT}_startup ${TARGET_ROBOT}eus --rosdistro melodic --exclude RPP --exclude mongodb_store | tee user.repos && \
    [ ${UPDATE_SOURCE_ROOT} -eq 0 -o -z \"\$(cat user.repos)\" ] || PYTHONPATH= vcs import src < user.repos && \
    export LDFLAGS=\"-L/home/nao/${INSTALL_ROOT}/ros1_inst/lib  -L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib \${LDFLAGS} -lpcre16 /home/nao/ctc/pcre/lib/libpcre.so.1 -lpcrecpp -lbz2 -llzma -ltiff -ljpeg -lpng -logg -licudata -licui18n -licuuc -lz -lassimp -lminizip -lxml2 -llzma -lqhull_p -lcurl -lssl -lcrypto -lgpg-error -lassuan -lgpgme -lyaml-cpp /home/nao/ctc/boost/lib/libboost_random-mt.so \" && \
    catkin $ARGS -s -vi \
        -DCMAKE_TOOLCHAIN_FILE=/home/nao/ctc-cmake-toolchain.cmake \
        -DALDE_CTC_CROSS=/home/nao/ctc \
        -DPYTHON_EXECUTABLE=/usr/bin/python2 \
        -DPYTHON_INCLUDE_DIR=/home/nao/System/Python-2.7.17/include/python2.7 \
        -DPYTHON_LIBRARY=/home/nao/System/Python-2.7.17/lib/libpython2.7.so \
        --cmake-args -DCATKIN_ENABLE_TESTING=FALSE --make-args VERBOSE=1\
    " 2>&1 | tee ${TARGET_MACHINE}_build_user.log
cp ${PWD}/startup_scripts/user_setup.bash ${SOURCE_ROOT}/

echo "

    export LDFLAGS=\"-L/home/nao/${INSTALL_ROOT}/ros1_inst/lib  -L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib \${LDFLAGS} -lpcre16 /home/nao/ctc/pcre/lib/libpcre.so.1 -lpcrecpp -lbz2 -llzma -ltiff -ljpeg -lpng -logg -licudata -licui18n -licuuc -lz -lassimp -lminizip -lxml2 -llzma -lqhull_p -lcurl -lssl -lcrypto \" && \

    catkin build jsk_${TARGET_ROBOT}_startup ${TARGET_ROBOT}eus  -s -vi \
    catkin build jsk_robot_startup --start-with jsk_topic_tools  -s -vi \
    catkin build naoqi_libqi --start-with naoqi_libqi  -s -vi \
    export CFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_inst/include -I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include \${CFLAGS}\" && \
    export CPPFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_inst/include -I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include \${CPPFLAGS}\" && \
    export CXXFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_inst/include -I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include  \${CXXFLAGS}\" && \
    export LDFLAGS=\"-L/home/nao/${INSTALL_ROOT}/ros1_inst/lib -L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib \${LDFLAGS} -lpcre16 /home/nao/ctc/pcre/lib/libpcre.so.1 -lpcrecpp  -lbz2 -llzma -ltiff -ljpeg -lpng -logg -licudata -licui18n -licuuc -lz -lassimp -lminizip -lxml2 -llzma -lqhull_p -lcurl -lssl -lcrypto \" && \
"
