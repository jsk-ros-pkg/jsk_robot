#!/bin/bash


# ros1_deendenceis
export CMAKE_PREFIX_PATH="/opt/jsk/System/ros1_dependencies:${CMAKE_PREFIX_PATH}"
export PKG_CONFIG_PATH="/opt/jsk/System/ros1_dependencies/lib/pkgconfig:${PKG_CONFIG_PATH}"
export LD_LIBRARY_PATH="/opt/jsk/System/ros1_dependencies/lib:${LD_LIBRARY_PATH}"
# Python's sys.path is automatically set in /usr/lib/python2.7/sitecustomize.py
# export PYTHONPATH="/opt/jsk/System/ros1_dependencies/lib/python2.7/site-packages:${PYTHONPATH}"

# GI : for gir1.2-gstreamer-1.0, which is installed by ros1_dependencies_build_scripts/0006-gstreamer
export GI_TYPELIB_PATH="/opt/jsk/System/ros1_dependencies/lib/girepository-1.0"

# Python
export LD_LIBRARY_PATH="/opt/jsk/System/Python/lib:${LD_LIBRARY_PATH}"
# Python's sys.path is automatically set in /usr/lib/python2.7/sitecustomize.py
# export PYTHONPATH="/opt/jsk/System/Python/lib/python2.7/site-packages:${PYTHONPATH}"
export PATH="/opt/jsk/System/Python/bin:${PATH}"


