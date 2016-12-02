#!/bin/sh



# head_mount_kinect_ir_optical_frame -> openni_ir_optical_frame
# head_mount_kinect_rgb_optical_frame -> openni_rgb_optical_frame
from_file=$1
to_file=$2
if [ ! -e "$from_file" ]; then
    echo "Cannot find from file: $from_file"
    exit 1
fi

echo "Converting kinect frame id from $1 to $2"
sed  "s/head_mount_kinect_ir_optical_frame/openni_ir_optical_frame/g" $from_file | sed  "s/head_mount_kinect_rgb_optical_frame/openni_rgb_optical_frame/g" >  $to_file
