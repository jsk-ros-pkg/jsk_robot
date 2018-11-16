#!/bin/bash
PKG_KEY="pkg_root:="
PKG_DEST=$(rospack find jsk_aero_startup)
ARGS=""
for OPT in "$@"; do
    case "$OPT" in
        __*)  # remove ROS arguments
            shift
            ;;
        pkg_root*)  # parse pkg root in caller environment
            PKG_ORIG=${1#$PKG_KEY}
	    shift
	    ;;
        *)
            ARGS="$ARGS ${1//$PKG_ORIG/$PKG_DEST}"
            shift
            ;;
    esac
done

roslaunch aero_startup wheel_with_static_map.launch $ARGS --screen 1>&2
