#!/bin/bash

## see https://dev.bostondynamics.com/docs/python/fetch_tutorial/fetch1

set -x
rm -fr dogtoy/images/*

trap "exit" INT TERM ERR
trap "kill 0" EXIT

python capture_images.py 10.0.0.3 --image-source frontleft_fisheye_image --folder dogtoy/images &
python capture_images.py 10.0.0.3 --image-source frontright_fisheye_image --folder dogtoy/images &
python capture_images.py 10.0.0.3 --image-source left_fisheye_image --folder dogtoy/images &
python capture_images.py 10.0.0.3 --image-source right_fisheye_image --folder dogtoy/images &
python capture_images.py 10.0.0.3 --image-source back_fisheye_image --folder dogtoy/images &

wait
