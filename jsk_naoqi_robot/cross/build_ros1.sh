#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-i386}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System
SOURCE_ROOT=${TARGET_MACHINE}_ws_system

PYTHON2_VERSION=2.7.17

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

ARGS=$@

set -xeuf -o pipefail

# put ros1_dependencies.bash
mkdir -p ${HOST_INSTALL_ROOT}
cp ${PWD}/startup_scripts/ros1_dependencies_setup.bash ${HOST_INSTALL_ROOT}

# generate pr2eus package and install them within ${SOURCE_ROOT}
mkdir -p ${SOURCE_ROOT}/src
mkdir -p ${HOST_INSTALL_ROOT}/ros1_inst

if [ ${UPDATE_SOURCE_ROOT} -eq 1 -o ! -e ${SOURCE_ROOT}/.imported ]; then
    vcs import --force --retry 3 --shallow ${SOURCE_ROOT}/src < repos/roseus_no_window.repos
    for dir in euslisp jskeus; do ls ${SOURCE_ROOT}/src/$dir/patches/; rsync -avz ${SOURCE_ROOT}/src/$dir/patches/ ${SOURCE_ROOT}/src/$dir; done
    sed -i s@:{version}@0.0.0@ ${SOURCE_ROOT}/src/euslisp/package.xml ${SOURCE_ROOT}/src/jskeus/package.xml
    sed -i 's@(\(need_catkin AND.*\))@(1)@' ${SOURCE_ROOT}/src/euslisp/CMakeLists.txt
    sed -i 's@(\(need_catkin AND.*\))@(1)@' ${SOURCE_ROOT}/src/jskeus/CMakeLists.txt
    sed -i 's@/usr/include /usr/X11R6/include@@' ${SOURCE_ROOT}/src/jsk_roseus/roseus/CMakeLists.txt
    grep include_directories ${SOURCE_ROOT}/src/jsk_roseus/roseus/CMakeLists.txt
    sed -i 's@CC=cc@CC=/home/nao/ctc/bin/i686-aldebaran-linux-gnu-cc@' ${SOURCE_ROOT}/src/euslisp/lisp/Makefile.Linux.thread
    grep CC ${SOURCE_ROOT}/src/euslisp/lisp/Makefile.Linux.thread
    sed -i 's@CC  ?= gcc@CC=/home/nao/ctc/bin/i686-aldebaran-linux-gnu-gcc@' ${SOURCE_ROOT}/src/jskeus/irteus/Makefile.Linux
    sed -i 's@CXX ?= g++@CXX=/home/nao/ctc/bin/i686-aldebaran-linux-gnu-g++@' ${SOURCE_ROOT}/src/jskeus/irteus/Makefile.Linux
    grep CC ${SOURCE_ROOT}/src/jskeus/irteus/Makefile.Linux
    grep CXX ${SOURCE_ROOT}/src/jskeus/irteus/Makefile.Linux
    #sed -i 's@^CFLAGS=\$(WFLAGS)@CFLAGS=-I/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/usr/include \$\(WFLAGS\)@' ${SOURCE_ROOT}/src/euslisp/lisp/Makefile.Linux.thread
    #sed -i 's@\$(CC) -o \$(BINDIR)/gccls@\$(CC) -I/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/usr/include -L/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/usr/lib -o \$(BINDIR)/gccls@' ${SOURCE_ROOT}/src/euslisp/lisp/Makefile.generic2
    #sed -i 's@^CFLAGS=-O2@CFLAGS=-I/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/usr/include -O2@' ${SOURCE_ROOT}/src/jskeus/irteus/Makefile.Linux
fi
#wget https://patch-diff.githubusercontent.com/raw/PR2/pr2_mechanism/pull/346.diff -O ${SOURCE_ROOT}/pr2_mechanism-346.diff

EUSCOLLADA_DEPENDS="assimp_devel collada_parser collada_urdf resource_retriever"
ROSEUS_DEPENDS="actionlib_tutorials"
ROSEUS_SMACH_DEPENDS="smach_ros"
ROSEUS_MONGO_DEPENDS="mongodb_store_msgs"  # mongodb_store is ignored
JSK_ROBOT_STARTUP_DEPENDS="jsk_topic_tools pr2_mechanism_controllers gmapping jsk_recognition_msgs pointcloud_to_laserscan posedetection_msgs rosbridge_server roswww tf2_geometry_msgs jsk_tilt_laser"  # app_manager will be installed in User space, mongodb_store is ignored
JSK_ROBOT_UTILS="jsk_network_tools"
DIAGNOSTIC_AGGREGATOR="diagnostic_aggregator"  # jsk_XXX_startup usually depends on diagnostic_aggregator
GEOMETRIC_SHAPES="octomap eigen_stl_containers random_numbers shape_msgs"
IMAGE_PIPELINE="camera_calibration_parsers image_geometry nodelet_topic_tools camera_info_manager stereo_msgs"
JSK_VISUALIZATION="jsk_hark_msgs jsk_gui_msgs people_msgs view_controller_msgs interactive_markers pcl_conversions pcl_ros"
PR2EUS="pr2eus"

# UPDATE_SOURCE_ROOT=1 #####################
docker run -it --rm \
  -u $(id -u $USER) \
  -e ALDE_CTC_CROSS=/home/nao/ctc \
  -v ${ALDE_CTC_CROSS}:/home/nao/ctc:ro \
  -v ${PWD}/ctc-cmake-toolchain.cmake:/home/nao/ctc-cmake-toolchain.cmake:ro \
  -e INSTALL_ROOT=${INSTALL_ROOT} \
  -v ${HOST_INSTALL_ROOT}/Python-${PYTHON2_VERSION}:/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/home/nao/${INSTALL_ROOT}/ros1_dependencies:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies_setup.bash:/home/nao/${INSTALL_ROOT}/ros1_dependencies_setup.bash:ro \
  -v ${HOST_INSTALL_ROOT}/ros1_inst:/home/nao/${INSTALL_ROOT}/ros1_inst:rw \
  -v ${PWD}/${SOURCE_ROOT}:/home/nao/${SOURCE_ROOT}:rw \
  ros1-pepper:${TARGET_MACHINE} \
  bash -c "\
    env && \
    set -xeuf -o pipefail && \
    cd ${SOURCE_ROOT} && \
    [ ${UPDATE_SOURCE_ROOT} -eq 0 ] || LD_LIBRARY_PATH=  ROS_PACKAGE_PATH=src rosinstall_generator ${EUSCOLLADA_DEPENDS} ${ROSEUS_DEPENDS} ${ROSEUS_MONGO_DEPENDS} ${ROSEUS_SMACH_DEPENDS} ${JSK_ROBOT_STARTUP_DEPENDS} ${DIAGNOSTIC_AGGREGATOR} ${GEOMETRIC_SHAPES} ${IMAGE_PIPELINE} ${JSK_VISUALIZATION} ${PR2EUS} --verbose --deps --rosdistro melodic --exclude RPP --depend-type buildtool buildtool_export build run | tee pepper_ros1_system.repos && \
    if [ ${UPDATE_SOURCE_ROOT} -eq 1 -o ! -e .imported ]; then \
       LD_LIBRARY_PATH=  vcs import --force --retry 3 src < pepper_ros1_system.repos; \
       find ./src -iname CMakeLists.txt -exec sed -i 's@cmake_minimum_required(VERSION 3.7)@cmake_minimum_required(VERSION 2.8.3)@' \\{} \\; ; \
       find ./src -iname CMakeLists.txt -exec sed -i 's@cmake_minimum_required(VERSION 3.7.2)@cmake_minimum_required(VERSION 2.8.3)@' \\{} \\; ; \
       sed -i 's@^pkg_check_modules(PC_EIGEN REQUIRED eigen3)@#pkg_check_modules(PC_EIGEN REQUIRED eigen3)@' ./src/pr2_mechanism/pr2_mechanism_diagnostics/CMakeLists.txt; \
       touch .imported; \
    fi && \
    source /home/nao/System/ros1_dependencies_setup.bash && \
    export PKG_CONFIG_PATH=\"/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib/pkgconfig:/home/nao/ctc/qt/lib/pkgconfig\" && \
    export CFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include \${CFLAGS}\" && \
    export CPPFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include \${CPPFLAGS}\" && \
    export CXXFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include  \${CXXFLAGS}\" && \
    export LDFLAGS=\"-L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib \${LDFLAGS} -lpcre16 /home/nao/ctc/pcre/lib/libpcre.so.1 -lpcrecpp  -lbz2 -llzma -ltiff -ljpeg -lpng -logg -licudata -licui18n -licuuc -lz -lassimp -lminizip -lxml2 -llzma -lqhull_p -lcurl -lssl -lcrypto -lgpg-error -lassuan -lgpgme -lyaml-cpp -lpthread -lpcl_io_ply \" && \
    export LIBS=\"-L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib -lz\" && \
    /usr/bin/python ./src/catkin/bin/catkin_make_isolated $ARGS --install --install-space /home/nao/${INSTALL_ROOT}/ros1_inst -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCMAKE_TOOLCHAIN_FILE=/home/nao/ctc-cmake-toolchain.cmake \
        -DALDE_CTC_CROSS=/home/nao/ctc \
        -DPYTHON_EXECUTABLE=/usr/bin/python2 \
        -DPYTHON_INCLUDE_DIR=/home/nao/System/Python-2.7.17/include/python2.7 \
        -DPYTHON_LIBRARY=/home/nao/System/Python-2.7.17/lib/libpython2.7.so \
        -DCATKIN_ENABLE_TESTING=FALSE \
        -DUSE_VISUALIZATION=OFF \
        -DEUSLISP_WITHOUT_DISPLAY=TRUE -DDISABLE_DOCUMENTATION=1 --make-args VERBOSE=1 \
        " 2>&1 | tee ${TARGET_MACHINE}_build_ros1.log

cp ${PWD}/startup_scripts/system_setup.bash ${HOST_INSTALL_ROOT}/

echo "
    export CMAKE_PREFIX_PATH=\"/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot:\${CMAKE_PREFIX_PATH}\" && \
    export aLD_LIBRARY_PATH=/home/nao/ctc/openssl/lib:/home/nao/ctc/zlib/lib:/home/nao/ctc/curl/lib:/home/nao/ctc/c_ares/lib && \
    export LD_LIBRARY_PATH=/home/nao/ctc/qhull/lib:/home/nao/ctc/pcre/lib:\${LD_LIBRARY_PATH} && \
        -DCMAKE_TOOLCHAIN_FILE=/home/nao/ctc-cmake-toolchain.cmake \
        -DALDE_CTC_CROSS=/home/nao/ctc \

    [ ! -e pr2_mechanism-346.diff ] || OUT=\"\$(patch -p1 --forward --directory src/pr2_mechanism < pr2_mechanism-346.diff | tee /dev/tty)\" || echo \"\${OUT}\" | grep \"Skipping patch\" -q || (echo \"\$OUT\" && false) && \
        -DCMAKE_LIBRARY_PATH=/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/lib \
        -DRT_LIBRARY= \
        -DCMAKE_TOOLCHAIN_FILE=/home/nao/ctc-cmake-toolchain.cmake \
        -DALDE_CTC_CROSS=/home/nao/ctc \
"
