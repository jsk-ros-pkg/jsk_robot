#!/bin/bash

## see https://dev.bostondynamics.com/docs/python/fetch_tutorial/fetch1

python model_main_tf2.py --model_dir=dogtoy/models/dogtoy_ssd_resnet50_v1_fpn --pipeline_config_path=dogtoy/models/dogtoy_ssd_resnet50_v1_fpn/pipeline.config --num_train_steps=20000
# CUDA_VISIBLE_DEVICES="-1" python model_main_tf2.py --model_dir=dogtoy/models/dogtoy_ssd_resnet50_v1_fpn --pipeline_config_path=dogtoy/models/dogtoy_ssd_resnet50_v1_fpn/pipeline.config --checkpoint_dir=dogtoy/models/dogtoy_ssd_resnet50_v1_fpn
mkdir -p dogtoy/exported-models/dogtoy-model
python exporter_main_v2.py --input_type image_tensor --pipeline_config_path dogtoy/models/dogtoy_ssd_resnet50_v1_fpn/pipeline.config --trained_checkpoint_dir dogtoy/models/dogtoy_ssd_resnet50_v1_fpn/ --output_directory dogtoy/exported-models/dogtoy-model
