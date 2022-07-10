#!/bin/bash

#env | grep PATH
#return
# Python
export LD_LIBRARY_PATH="/home/nao/System/Python-2.7.17/lib:${LD_LIBRARY_PATH}"
#####export LD_LIBRARY_PATH="/home/nao/System/Python-2.7.17/lib"
export PYTHONPATH="/home/nao/System/Python-2.7.17/lib/python2.7/site-packages:${PYTHONPATH+${PYTHONPATH}}"  ## On Pepper's PC, PYTHONPATH is set to /opt/aldebaran/lib/python2.7/site-packages
export PATH="/home/nao/System/Python-2.7.17/bin:${PATH}"

# ros1_deendenceis
export CMAKE_PREFIX_PATH="/home/nao/System/ros1_dependencies:${CMAKE_PREFIX_PATH}"
#export CMAKE_LIBRARY_PATH="/home/nao/ctc/i686-aldebaran-linux-gnu/sysroot/lib:${CMAKE_LIBRARY_PATH}"
export PKG_CONFIG_PATH="/home/nao/System/ros1_dependencies/lib/pkgconfig"
export LD_LIBRARY_PATH="/home/nao/System/ros1_dependencies/lib:${LD_LIBRARY_PATH}"
#export PYTHONPATH="/home/nao/System/ros1_dependencies/lib/python2.7/site-packages:${PYTHONPATH}"

# GI : for gir1.2-gstreamer-1.0, which is installed by ros1_dependencies_build_scripts/0006-gstreamer
export GI_TYPELIB_PATH="/home/nao/System/ros1_dependencies/lib/girepository-1.0"

