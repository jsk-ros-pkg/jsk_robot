coreio extensions
-----------------

Please follow https://dev.bostondynamics.com/docs/python/fetch_tutorial/fetch1 to get images, annotate target, training network. After that, we expect following directory structure.

```
- dogtoy/
  |- annotations/
     |- label_map.pbtxt
  |- exported-models/
     |- dogtoy-model/
        |- checkpoint/
        |- pipeline.config
        |- saved_model/
  |- models/
  |- pre-trained-models/
```


Then copy `dogtoy-model` directory within `dogtoy/exported-models/` to this directory and put `annotations/label_map.pbtxt` under `dogtoy-model`. See `Makefile` for detail.

To build docker run `make all`. To export docker image, run `make export`. After that copy `object_detection_coco.tar.gz` to `/opt/` directory of SpotCORE. To copy the data, we recommend to connect the ethernet cable to the robot or dock and use `scp -P 20022 object_detection_coco.tar.gz spot@10.0.0.3:/tmp/` to copy data and then, login to to spotcore and run `sudo cp /tmp/object_detection_coco.tar.gz /opt`. Please keep backup before you overwrite `tar.gz` file.

