#!/bin/sh

OUTPUT_FILE=$1

# generating pr2 urdf
INPUT_XACRO=`rospack find pr2_description`/robots/pr2.urdf.xacro
INPUT_URDF=`mktemp`
rosrun xacro xacro.py $INPUT_XACRO > $INPUT_URDF

# original urdf
# child(1):  head_mount_link
#     child(1):  head_mount_kinect_ir_link
#         child(1):  head_mount_kinect_ir_optical_frame
#         child(2):  head_mount_kinect_rgb_link
#             child(1):  head_mount_kinect_rgb_optical_frame

# JSK expects openni_rgb_optical_frame and openni_ir_optical_frame frames
# 
# child(1):  head_mount_link
#     child(1):  head_mount_kinect_ir_link
#         child(1):  head_mount_kinect_rgb_link
#             child(1):  openni_rgb_optical_frame
#         child(2):  openni_ir_optical_frame

# generated urdf
# child(1):  head_mount_link
#     child(1):  head_mount_kinect_ir_link
#         child(1):  head_mount_kinect_ir_optical_frame --> openni_ir_optical_frame
#         child(2):  head_mount_kinect_rgb_link --> openni_rgb_frame
#             child(1):  head_mount_kinect_rgb_optical_frame --> openni_rgb_optical_frame

# we will add 3 more frames
tmp0=`mktemp`
tmp1=`mktemp`
tmp2=`mktemp`
rosrun euscollada add_sensor_to_urdf.py 0 0 0 0 0 0 head_mount_kinect_ir_optical_frame openni_ir_optical_frame $INPUT_URDF $tmp0
rosrun euscollada add_sensor_to_urdf.py 0 0 0 0 0 0 head_mount_kinect_rgb_link openni_rgb_frame $tmp0 $tmp1
rosrun euscollada add_sensor_to_urdf.py 0 0 0 0 0 0 head_mount_kinect_rgb_optical_frame openni_rgb_optical_frame $tmp1 $tmp2
cp $tmp2 $OUTPUT_FILE
