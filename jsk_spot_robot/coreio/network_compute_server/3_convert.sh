#!/bin/bash

## see https://dev.bostondynamics.com/docs/python/fetch_tutorial/fetch1

python generate_tfrecord.py --xml_dir dogtoy/annotations/train --image_dir dogtoy/images --labels_path dogtoy/annotations/label_map.pbtxt --output_path dogtoy/annotations/train.record
python generate_tfrecord.py --xml_dir dogtoy/annotations/test --image_dir dogtoy/images --labels_path dogtoy/annotations/label_map.pbtxt --output_path dogtoy/annotations/test.record

