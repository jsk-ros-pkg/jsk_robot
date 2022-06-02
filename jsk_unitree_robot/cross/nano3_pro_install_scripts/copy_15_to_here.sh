#!/bin/bash

set -euf -o pipefail

ssh-keygen -f "${HOME}/.ssh/known_hosts" -R "192.168.123.15" || echo "OK"
sshpass -p 123 ssh -o StrictHostKeyChecking=no unitree@192.168.123.15 exit

LOCAL_DATA=15-local.tgz # 2.7G, before remove Cyton.jtop,matplotlib,mpl_toolkit,opencv except dnn
LOCAL_LOGS=15-local.txt
# gunzip 15-local.tgz; tar -f 15-local.tar --wildcards --delete 'usr/lib/aarch64-linux-gnu/*.a'  --delete 'usr/local/cuda-10.2/targets/aarch64-linux/lib/*.a'
# gzip < 15-local.tar > 15-local.tgz

ssh unitree@192.168.123.15 "tar -cvz \
    --exclude /usr/local/cuda-10.2/samples \
    --exclude /usr/local/cuda-10.2/doc \
    --exclude /usr/local/cuda-10.2/nvvm/libnvvm-samples/ \
    --exclude /usr/local/cuda-10.2/targets/aarch64-linux/include \
    --exclude /usr/local/cuda-10.2/include \
    --exclude /usr/local/cuda-10.2/extras \
    --exclude /usr/local/cuda-10.2/tools \
    --exclude /usr/local/cuda-10.2/share \
    --exclude /usr/local/cuda-10.2/nvvmx \
    --exclude /usr/local/cuda-10.2/nvvm \
    --exclude /usr/local/cuda-10.2/bin \
    --exclude /home/unitree/.local/lib/python3.6/site-packages/torch/test \
    --exclude /usr/local/jetson_stats \
    --exclude /usr/local/lib/cmake \
    --exclude /usr/local/lib/libpaho* \
    --exclude '/usr/local/lib/python3.6/dist-packages/Cython*' \
    --exclude /usr/local/lib/python3.6/dist-packages/jtop \
    --exclude '/usr/local/lib/python3.6/dist-packages/mpl_toolkits*' \
    --exclude /usr/local/man \
    --exclude /usr/local/share \
    --exclude /usr/local/sbin \
    --exclude /usr/local/src \
    --exclude /usr/local/games \
    --exclude 'usr/lib/aarch64-linux-gnu/*.a' \
    --exclude 'usr/local/cuda-10.2/targets/aarch64-linux/lib/*.a' \
    /usr/lib/python3.6/dist-packages \
    /usr/lib/aarch64-linux-gnu/libnv* \
    /usr/lib/aarch64-linux-gnu/libcud* \
    /usr/lib/aarch64-linux-gnu/libmyelin* \
    /usr/lib/aarch64-linux-gnu/tegra \
    /etc/alternatives/libcud* \
    /usr/lib/aarch64-linux-gnu/libopencv_dnn* \
    /usr/lib/aarch64-linux-gnu/libcublas* \
    /usr/lib/aarch64-linux-gnu/libtbb* \
    /usr/lib/aarch64-linux-gnu/libopenblas*  \
    /etc/ld.so.conf.d/cuda-10-2.conf \
    /usr/local/ \
    ~/.local/bin \
    ~/.local/lib \
    ~/.cache/torch \
    /home/unitree/Unitree/autostart/imageai/mLComSystemFrame/pyScripts/human_pose_trt.pth \
" 2> >(tee ${LOCAL_LOGS}) > ${LOCAL_DATA}
#
