#!/bin/bash -ex

## https://dev.bostondynamics.com/docs/python/fetch_tutorial/fetch6
## https://dev.bostondynamics.com/docs/python/fetch_tutorial/files/coreio_extension/create_extension.sh

mkdir -p data
rm -rf data/*
# Dogtoy model
cp -r ./dogtoy/exported-models/dogtoy-model data/.
# and its label map
cp ./dogtoy/annotations/label_map.pbtxt data/dogtoy-model/.
# coco model (includes its label map)
cp -r ./dogtoy/pre-trained-models/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8 data/.
cp ./models-with-protos/research/object_detection/data/mscoco_label_map.pbtxt data/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8/

# Build the image
docker build -t object_detection_coco:l4t -f Dockerfile.l4t .

# Exports the image, uses pigz
docker save object_detection_coco:l4t | pigz > object_detection_coco_image.tar.gz

# Built the Spot Extension by taring all the files together
tar -cvzf object_detection_coco.spx \
    object_detection_coco_image.tar.gz \
    manifest.json \
    docker-compose.yml \
    data

# Cleanup intermediate image
rm object_detection_coco_image.tar.gz
rm -fr data
