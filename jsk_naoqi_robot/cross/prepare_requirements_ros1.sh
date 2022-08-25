#!/bin/bash

TARGET_MACHINE="${TARGET_MACHINE:-i386}"
HOST_INSTALL_ROOT="${BASE_ROOT:-${PWD}}/"${TARGET_MACHINE}_System
INSTALL_ROOT=System

PYTHON2_VERSION=2.7.17

set -xeuf -o pipefail

if [ -z "${ALDE_CTC_CROSS}" ]; then
  echo "Please define the ALDE_CTC_CROSS variable with the path to Aldebaran's Crosscompiler toolchain"
  exit 1
fi
# setup ros1-unitree docker, which installed package also installed in the robot
if ! docker buildx > /dev/null; then
    DOCKER_BUILDKIT=1 docker build --platform=local -o . "https://github.com/docker/buildx.git"
fi
if [ -e /proc/sys/fs/binfmt_misc/qemu-aarch64 ] &&  [ ! "$(docker images -q multiarch/qemu-user-static)" ]; then
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
fi

docker buildx build $@ --progress plain -t ros1-pepper:${TARGET_MACHINE} --build-arg TARGET_MACHINE=${TARGET_MACHINE} -f docker/Dockerfile_ros1 docker/ 2>&1 | tee ${TARGET_MACHINE}_prepare_requirements_ros1.log

if [ ! -e "Python-${PYTHON2_VERSION}.tar.xz" ]; then
  wget -cN https://www.python.org/ftp/python/$PYTHON2_VERSION/Python-${PYTHON2_VERSION}.tar.xz
fi
if [ ! -e "Python-${PYTHON2_VERSION}" ]; then
    tar xvf Python-${PYTHON2_VERSION}.tar.xz
fi

if [ ! -e "zlib_1.2.11.dfsg.orig.tar.xz" ]; then
    wget -cN http://archive.ubuntu.com/ubuntu/pool/main/z/zlib/zlib_1.2.11.dfsg.orig.tar.xz # override ctc version(1.2.8)
fi
if [ ! -e "zlib-1.2.11" ]; then
    tar -xvJf zlib_1.2.11.dfsg.orig.tar.xz
fi

if [ ! -e "minizip_1.1.orig.tar.xz" ]; then
    wget -cN http://archive.ubuntu.com/ubuntu/pool/universe/m/minizip/minizip_1.1.orig.tar.xz
fi
if [ ! -e "minizip_1.1-8build1.debian.tar.xz" ]; then
    wget -cN http://archive.ubuntu.com/ubuntu/pool/universe/m/minizip/minizip_1.1-8build1.debian.tar.xz
fi
if [ ! -e "minizip" ]; then
    tar -xvJf minizip_1.1.orig.tar.xz
fi
if [ ! -e "minizip/debian" ]; then
    tar -xvJf minizip_1.1-8build1.debian.tar.xz -C minizip
    (cd minizip/debian/patches; for patch_file in $(cat series); do patch -p1 --directory ../../ < ${patch_file}; done)
fi

if [ ! -e "python-lzma_0.5.3.orig.tar.bz2" ]; then
    wget -cN http://archive.ubuntu.com/ubuntu/pool/universe/p/python-lzma/python-lzma_0.5.3.orig.tar.bz2
fi
if [ ! -e "python-lzma_0.5.3-3.debian.tar.xz" ]; then
    wget -cN http://archive.ubuntu.com/ubuntu/pool/universe/p/python-lzma/python-lzma_0.5.3-3.debian.tar.xz
fi
if [ ! -e "pyliblzma-0.5.3" ]; then
    tar -xvjf python-lzma_0.5.3.orig.tar.bz2
fi
if [ ! -e "pyliblzma-0.5.3/debian" ]; then
    tar -xvJf python-lzma_0.5.3-3.debian.tar.xz -C pyliblzma-0.5.3
fi
#
mkdir -p ${HOST_INSTALL_ROOT}/ros1_dependencies
mkdir -p ${HOST_INSTALL_ROOT}/Python-${PYTHON2_VERSION}
mkdir -p ${PWD}/Python-${PYTHON2_VERSION}-host

docker run -it --rm \
  -u $(id -u $USER) \
  -e PYTHON2_VERSION=${PYTHON2_VERSION} \
  -v ${PWD}/Python-${PYTHON2_VERSION}:/home/nao/Python-${PYTHON2_VERSION}-src \
  -v ${PWD}/Python-${PYTHON2_VERSION}-host:/home/nao/Python-${PYTHON2_VERSION}-host \
  -v ${PWD}/zlib-1.2.11:/home/nao/zlib-1.2.11-src \
  -v ${PWD}/minizip:/home/nao/minizip-src \
  -v ${PWD}/pyliblzma-0.5.3:/home/nao/pyliblzma-0.5.3-src \
  -v ${HOST_INSTALL_ROOT}/Python-${PYTHON2_VERSION}:/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION} \
  -v ${HOST_INSTALL_ROOT}/ros1_dependencies:/home/nao/${INSTALL_ROOT}/ros1_dependencies:rw \
  -v ${ALDE_CTC_CROSS}:/home/nao/ctc \
  ros1-pepper:${TARGET_MACHINE} \
  bash -c "\
    env; echo set -xeuf -o pipefail && \
    \
    echo ';; Compile zlib' && \
    \
    mkdir -p /home/nao/zlib-1.2.11-src/build && \
    cd /home/nao/zlib-1.2.11-src/build && \
    cmake \
        -DCMAKE_INSTALL_PREFIX=/home/nao/${INSTALL_ROOT}/ros1_dependencies \
        -DCMAKE_BUILD_TYPE=Release \
        -DINSTALL_PKGCONFIG_DIR=/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib/pkgconfig \
        .. && \
    make -j4 install VERBOSE=1 && \
    \
    echo ';; Compile minizip' && \
    \
    export LD_LIBRARY_PATH=\"/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib:\${LD_LIBRARY_PATH}\" &&
    export PKG_CONFIG_PATH=\"/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib/pkgconfig\" &&
    export CFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include \${CFLAGS}\" &&
    export CPPFLAGS=\"-I/home/nao/${INSTALL_ROOT}/ros1_dependencies/include \${CPPFLAGS}\" &&
    export LDFLAGS=\"-L/home/nao/${INSTALL_ROOT}/ros1_dependencies/lib \${LDFLAGS}\" &&
    cd /home/nao/minizip-src &&
    autoreconf -vfi &&
    ./configure \
         --prefix=/home/nao/${INSTALL_ROOT}/ros1_dependencies \
         --host=i686-aldebaran-linux-gnu \
         --build=i686-linux-gnu \
         --enable-shared &&
    make -j4 install VERBOSE=1 && \
    \
    echo ';; Compile Python' && \
    \
    export LD_LIBRARY_PATH=\"/home/nao/ctc/openssl/lib:/home/nao/ctc/zlib/lib:/home/nao/Python-${PYTHON2_VERSION}-host/lib\" &&
    export PATH=\"/home/nao/Python-${PYTHON2_VERSION}-host/bin:\${PATH}\" &&
    mkdir -p /home/nao/Python-${PYTHON2_VERSION}-src/build && \
    cd /home/nao/Python-${PYTHON2_VERSION}-src/build && \
    ../configure \
      --with-universal-arch=64-bit \
      --prefix=/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION} \
      --host=i686-aldebaran-linux-gnu \
      --build=x86_64-linux \
      --with-system-ffi \
      --enable-shared \
      --disable-ipv6 \
      --enable-unicode=ucs4 \
      ac_cv_file__dev_ptmx=yes \
      ac_cv_file__dev_ptc=no && \
    make -j4 VERBOSE=1 && \
    export LD_LIBRARY_PATH=\"/home/nao/ctc/openssl/lib:/home/nao/ctc/zlib/lib:/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/lib\" &&
    export PATH=\"/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/bin:\${PATH}\" &&
    make install VERBOSE=1 && \
    \
    echo ';; Install pip' && \
    \
    wget -O - -q https://bootstrap.pypa.io/pip/2.7/get-pip.py | python && \
    pip install cython==0.26.1 && \
    pip install PyYaml==3.12 Pillow==5.1.0 empy==3.3.2 docutils==0.14 pyparsing==2.2.0 python-dateutil==2.6.1 six==1.11.0 setuptools==39.0.1 numpy==1.13.3 defusedxml==0.5.0 netifaces==0.10.4 pymongo==3.6.1 image tornado==4.5.3 && \
    pip install catkin-pkg==0.5.2 rospkg==1.4.0 catkin-tools==0.6.1 && \
    pip install service-identity==16.0.0 pyasn1==0.4.2 pyasn1-modules==0.2.1 && \
    pip install pycryptodomex==3.4.7 && \
    \
    echo ';; Install Twisted' && \
    \
    pip install pyOpenSSL==17.5.0 cryptography==2.1.4 enum34==1.1.6 ipaddress==1.0.17  cffi==1.11.5 pycparser==2.18 asn1crypto==0.24.0 idna==2.6&& \
    cd /home/nao/Python-${PYTHON2_VERSION}-src/build && \
    wget https://files.pythonhosted.org/packages/93/ba/83fe91c2f6c613d53ce65d89b50620fc760d698a2f66b6b80147118a5c0f/Twisted-16.0.0.tar.bz2 && \
    tar -xjvf Twisted-16.0.0.tar.bz2 && \
    cd Twisted-16.0.0 && \
    python setup.py install
    \
    echo ';; Install rosinstall_generator' && \
    \
    cd /home/nao/Python-${PYTHON2_VERSION}-src/build && \
    if [ ! -e /tmp/rosinstall_generator ]; then git clone https://github.com/k-okada/rosinstall_generator /tmp/rosinstall_generator -b add_depend_type; fi && \
    pip install /tmp/rosinstall_generator && \
    \
    pip install rosdep==0.22.1 rosdistro==0.9.0 \
    " | tee -a ${TARGET_MACHINE}_prepare_requirements_ros1.log


echo "
    \
    echo ';; Install rosinstall_generator' && \
    \
    cd /home/nao/Python-${PYTHON2_VERSION}-src/build && \
    git clone https://github.com/k-okada/rosinstall_generator /tmp/rosinstall_generator -b add_depend_type && \
    pip install /tmp/rosinstall_generator \

    echo \"Could not remove -I/usr/incldue/i386-linux-gnu (k-okada)\" && sudo mv /usr/include/i386-linux-gnu /usr/include/i386-linux-gnu_ && \

    CFLAGS=\"-I/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/usr/include -I/home/nao/ctc/bzip2/include -I/home/nao/ctc/openssl/include -I/home/nao/ctc/xml2/include -I/home/nao/ctc/lz4/include -I/home/nao/ctc/ogg/include -I/home/nao/ctc/ffi/lib/libffi-3.0.13/include/\" &&
    CPPFLAGS=\"-I/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/usr/include -I/home/nao/ctc/bzip2/include -I/home/nao/ctc/openssl/include -I/home/nao/ctc/xml2/include -I/home/nao/ctc/lz4/include -I/home/nao/ctc/ogg/include -I/home/nao/ctc/ffi/lib/libffi-3.0.13/include/\" &&
    CXXFLAGS=\"-I/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/usr/include -I/home/nao/ctc/bzip2/include -I/home/nao/ctc/openssl/include -I/home/nao/ctc/xml2/include -I/home/nao/ctc/lz4/include -I/home/nao/ctc/ogg/include -I/home/nao/ctc/ffi/lib/libffi-3.0.13/include/\" &&
    LDFLAGS=\"-L/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/lib \${LDFLAGS}\" &&

      --with-system-ffi \
    aclocal && \
    libtoolize && \
    automake --add-missing && \
    autoconf && \

    /home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/bin/pip install catkin-tools && \
    \
    echo ';; Install lzma' && \
    \
    LDFLAGS=\"\${LDFLAGS} -llzma\" pip install /home/nao/pyliblzma-0.5.3-src && \
"
exit

docker run -it --rm \
  -u $(id -u $USER) \
  -e PYTHON2_VERSION=${PYTHON2_VERSION} \
  -v ${PWD}/Python-${PYTHON2_VERSION}:/home/nao/Python-${PYTHON2_VERSION}-src \
  -v ${PWD}/Python-${PYTHON2_VERSION}-host:/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION} \
  -e CC \
  -e CPP \
  -e CXX \
  -e RANLIB \
  -e AR \
  -e AAL \
  -e LD \
  -e READELF \
  -e CFLAGS \
  -e CPPFLAGS \
  -e LDFLAGS \
  ros1-pepper:${TARGET_MACHINE} \
  bash -c "\
exit;    set -euf -o pipefail && \
    wget https://github.com/dvargasfr/artifacts/raw/master/bzip2-1.0.6/bzip2-1.0.6.tar.gz && \
    tar -xvf bzip2-1.0.6.tar.gz && \
    cd bzip2-1.0.6 && \
    make -f Makefile-libbz2_so && \
    make && \
    make install PREFIX=/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION} && \
    cd .. && \
    mkdir -p /home/nao/Python-${PYTHON2_VERSION}-src/build-host && \
    cd /home/nao/Python-${PYTHON2_VERSION}-src/build-host && \
    LT_SYS_LIBRARY_PATH= ../configure \
      --prefix=/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION} \
      --disable-ipv6 \
      --enable-unicode=ucs4 \
      ac_cv_file__dev_ptmx=yes \
      ac_cv_file__dev_ptc=no && \
    export LD_LIBRARY_PATH=\"/home/nao/ctc/openssl/lib:/home/nao/ctc/zlib/lib:/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/lib\" && \
    export PATH=\"/home/nao/${INSTALL_ROOT}/Python-${PYTHON2_VERSION}/bin:\${PATH}\" &&
    make -j4 install VERBOSE=1 && \
    \
    wget -O - -q https://bootstrap.pypa.io/pip/2.7/get-pip.py | python && \
    pip install PyYaml==3.12 Pillow==5.1.0 empy==3.3.2 docutils==0.14 pyparsing==2.2.0 python-dateutil==2.6.1 six==1.11.0 catkin-pkg==0.5.2 setuptools==39.0.1 numpy==1.13.3 rospkg==1.4.0 defusedxml==0.5.0 netifaces==0.10.4 pymongo==3.6.1 image tornado==4.5.3 catkin-tools==0.6.1 && \
    pip install service-identity==16.0.0 pyasn==0.4.2 && \
    pip install vcstool==0.3.0 && \
    cd /home/nao/Python-${PYTHON2_VERSION}-src/build-host && \
    git clone https://github.com/k-okada/rosinstall_generator /tmp/rosinstall_generator -b add_depend_type && \
    pip install /tmp/rosinstall_generator \
    \
    " | tee -a ${TARGET_MACHINE}_prepare_requirements_ros1.log


