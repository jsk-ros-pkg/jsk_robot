#!/bin/bash

## see https://dev.bostondynamics.com/docs/python/fetch_tutorial/fetch1

python network_compute_server.py -m dogtoy/exported-models/dogtoy-model/saved_model dogtoy/annotations/label_map.pbtxt 10.0.0.3

