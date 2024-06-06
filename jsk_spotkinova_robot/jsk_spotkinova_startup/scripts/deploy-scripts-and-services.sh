#!/bin/bash

PACKAGE_PATH=$(rospack find jsk_spotkinova_startup)
SERVICE_SOURCE_PATH=$PACKAGE_PATH/services
SERVICE_DESTINATION_PATH=/etc/systemd/system

# install systemd unit files
cd $SERVICE_SOURCE_PATH
for service_file in $(ls ./*);
do
    sudo cp $service_file $SERVICE_DESTINATION_PATH/$service_file
    sudo chmod 644 $SERVICE_DESTINATION_PATH/$service_file
    sudo chown root:root $SERVICE_DESTINATION_PATH/$service_file
done
sudo systemctl daemon-reload
