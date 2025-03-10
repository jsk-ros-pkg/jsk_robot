# Use a base image provided by nvidia that already contains tensorflow 2.5
## FROM nvcr.io/nvidia/l4t-tensorflow:r32.6.1-tf2.5-py3
## https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html#framework-matrix-2019
FROM nvcr.io/nvidia/tensorflow:19.10-py3


# Do some basic apt and pip updating
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip libgl1 &&\
    apt-get clean

# Copy over the python requirements file and our prebuilt models API library
COPY docker-requirements.txt prebuilt/*.whl ./
COPY models-with-protos models-with-protos


# Install the python requirements
RUN python3 -m pip install pip==21.3.1 setuptools==59.6.0 wheel==0.37.1 && \
    python3 -m pip install -r docker-requirements.txt --find-links .

# https://dev.bostondynamics.com/docs/python/fetch_tutorial/fetch2
RUN python3 -m pip install opencv-python==4.5.*
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install tensorflow-gpu==2.3.1 tensorflow==2.3.1 tensorboard==2.3.0 tf-models-official==2.3.0 pycocotools lvis && \
    python3 -m pip uninstall -y opencv-python-headless


RUN python3 -m pip install models-with-protos/research

# copy pre-trained model
COPY ssd_resnet50_v1_fpn_640x640_coco17_tpu-8 /data/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8
RUN  cp models-with-protos/research/object_detection/data/mscoco_label_map.pbtxt /data/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8/

# copy trained model
COPY dogtoy-model /data/dogtoy-model

# Copy over our main script
COPY network_compute_server.py /app/
WORKDIR /app

# Set our script as the main entrypoint for the container
ENTRYPOINT ["python3", "network_compute_server.py"]
