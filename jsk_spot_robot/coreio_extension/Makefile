HOST_IP=192.168.50.3 # Or 10.0.0.3

all:
	[ -e models-with-protos.zip ] || wget https://dev.bostondynamics.com/docs/python/fetch_tutorial/files/models-with-protos.zip
	[ -e models-with-protos ] || unzip models-with-protos.zip
	[ -e ssd_resnet50_v1_fpn_640x640_coco17_tpu-8.tar.gz ] || wget http://download.tensorflow.org/models/object_detection/tf2/20200711/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8.tar.gz
	[ -e ssd_resnet50_v1_fpn_640x640_coco17_tpu-8 ] || tar -xvzf ssd_resnet50_v1_fpn_640x640_coco17_tpu-8.tar.gz
	[ -e dogtoy-model ] || cp -r ../dogtoy/exported-models/dogtoy-model .
	[ -e dogtoy-model/label_map.pbtxt ] || cp ../dogtoy/annotations/label_map.pbtxt dogtoy-model/
	docker build -t object_detection_coco -f Dockerfile .

run:
	docker run --network host \
	-v /opt/payload_credentials/payload_guid_and_secret:/opt/payload_credentials/payload_guid_and_secret \
	--rm object_detection_coco \
	-m /data/dogtoy-model/saved_model /data/dogtoy-model/label_map.pbtxt \
	-m /data/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8/saved_model \
	   /data/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8/mscoco_label_map.pbtxt \
	--payload-credentials-file /opt/payload_credentials/payload_guid_and_secret \
	--name ssd-resnet-coco-server \
	$(HOST_IP)

export:
	[ -e object_detection_coco.tar.gz ] || docker save object_detection_coco | pigz > object_detection_coco.tar.gz
	# scp -P 20022 *.tgz to spot@10.0.0.3:/opt
	# scp -P 20022 *.service spot@10.0.0.3:/lib/systemd/system/

bash:
	docker run --gpus all --rm -it  --entrypoint /bin/bash object_detection_coco
