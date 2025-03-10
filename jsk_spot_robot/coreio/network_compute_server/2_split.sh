#!/bin/bash

mv dogtoy/images/*.xml dogtoy/annotations/
python split_dataset.py --labels-dir dogtoy/annotations/ --output-dir dogtoy/annotations/ --ratio 0.9
