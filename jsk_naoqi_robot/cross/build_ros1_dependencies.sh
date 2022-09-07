#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-i386}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System
SOURCE_ROOT=${TARGET_MACHINE}_ws_ros1_dependencies_sources

PYTHON2_VERSION=2.7.17

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

if [ -z "$ALDE_CTC_CROSS" ]; then
  echo "Please define the ALDE_CTC_CROSS variable with the path to Aldebaran's Crosscompiler toolchain"
  exit 1
fi

# install development libraries requried for ros1 compile
mkdir -p ${HOST_INSTALL_ROOT}/ros1_dependencies
mkdir -p ${SOURCE_ROOT}/src
cp repos/ros1_dependencies.repos ${SOURCE_ROOT}/
cp repos/pepper_requirements.txt ${SOURCE_ROOT}/pepper_requirements.txt

docker run -it --rm \
  -u $(id -u $USER) \
  -e ALDE_CTC_CROSS=/home/nao/ctc \
  -v ${ALDE_CTC_CROSS}:/home/nao/ctc:ro \
  -e INSTALL_ROOT=${INSTALL_ROOT} \
  -v ${PWD}/ros1_dependencies_build_scripts:/home/nao/ros1_dependencies_build_scripts:ro \
  -v ${PWD}/${SOURCE_ROOT}:/home/nao/ros1_dependencies_sources:rw \
  -v ${HOST_INSTALL_ROOT}/Python-${PYTHON2_VERSION}:/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}:rw \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/home/nao/${INSTALL_ROOT}/ros1_dependencies:rw \
  ros1-pepper:${TARGET_MACHINE} \
  bash -c "\
    set -xeuf -o pipefail && \
    cd /home/nao/ros1_dependencies_sources && \
    if [ ! -e .imported ]; then LD_LIBRARY_PATH=  vcs import --skip-existing --force --retry 3 src < ros1_dependencies.repos; touch .imported; fi && \
    export PATH=\"/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/bin:/home/nao/${INSTALL_ROOT}/ros1_dependencies/bin:\${PATH}\" &&
    export LD_LIBRARY_PATH=\"/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/lib:/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib:\${LD_LIBRARY_PATH}\" && \
    export CFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include -I/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/include/python2.7 \${CFLAGS}\" && \
    export CPPFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include \${CPPFLAGS}\" && \
    export CXXFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include  \${CXXFLAGS}\" && \
    export LDFLAGS=\"-L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib -L/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/lib \${LDFLAGS} -lz -llzma -ltiff -ljpeg -lpng -logg\" && \
    export LIBS=\"-L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib -lz\" && \
    export PKG_CONFIG_PATH=\"/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib/pkgconfig\" && \
    export PYTHONPATH=\"/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib/python2.7/site-packages\" && \
    export CC=\"/home/nao/ctc/bin/i686-aldebaran-linux-gnu-cc\" && \
    export CXX=\"/home/nao/ctc/bin/i686-aldebaran-linux-gnu-c++\" && \
    for script_file in \$(ls /home/nao/ros1_dependencies_build_scripts/|sort); do
      /home/nao/ros1_dependencies_build_scripts/\$script_file || exit 1;
    done && \
    pip install --prefix=/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION} -r /home/nao/ros1_dependencies_sources/pepper_requirements.txt \
    " 2>&1 | tee ${TARGET_MACHINE}_build_ros1_dependencies.log

echo '
    export LD_LIBRARY_PATH=''; for dir in \$(find /home/nao/ctc/ -maxdepth 2 -iname lib -not -path '*/i686-aldebaran-linux-gnu/*'); do echo \${dir}; set +x; [ -d \${dir} ] && export LD_LIBRARY_PATH=\${dir}:\${LD_LIBRARY_PATH}; set -x; done && \
    export CMAKE_TOOLCHAIN_FILE=/home/nao/pepper_ros1_ws/ctc-cmake-toolchain.cmake && \
    export CMAKE_FIND_ROOT_PATH=\"/home/nao/${INSTALL_ROOT}/ros1_dependencies;/home/nao/ctc\" && \
    export CMAKE_C_FLAGS=\"-std=gnu11 -L/home/nao/ctc/icu/lib -L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib \" && \
    export CMAKE_CXX_FLAGS=\"-std=gnu++14 -L/home/nao/ctc/icu/lib -L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib \" && \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-gcc:/usr/bin/gcc:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-g++:/usr/bin/g++:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-c++:/usr/bin/c++:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-cc:/usr/bin/cc:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-cpp:/usr/bin/cpp:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-ld:/usr/bin/ld:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-ar:/usr/bin/ar:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-ranlib:/usr/bin/ranlib:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-nm:/usr/bin/nm:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-objcopy:/usr/bin/objcopy:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-objdump:/usr/bin/objdump:ro \
  -v ${ALDE_CTC_CROSS}/bin/i686-aldebaran-linux-gnu-strip:/usr/bin/strip:ro \
  -v ${ALDE_CTC_CROSS}/libexec/gcc/i686-aldebaran-linux-gnu/4.9.2/cc1:/usr/bin/cc1:ro \
  -v ${ALDE_CTC_CROSS}/libexec/gcc/i686-aldebaran-linux-gnu/4.9.2/cc1plus:/usr/bin/cc1plus:ro \
'
