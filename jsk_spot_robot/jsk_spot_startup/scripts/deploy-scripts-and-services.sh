#!/bin/bash

PACKAGE_PATH=$(rospack find jsk_spot_startup)
SCRIPT_DESTINATION_PATH=/var/lib/robot
SERVICE_DESTINATION_PATH=/etc/systemd/system

sudo cp $PACKAGE_PATH/scripts/update-network-connection.sh $SCRIPT_DESTINATION_PATH/update-network-connection.sh

sudo cp $PACKAGE_PATH/services/jsk-spot-network.service $SERVICE_DESTINATION_PATH/jsk-spot-network.service
sudo chmod 777 $SERVICE_DESTINATION_PATH/jsk-spot-network.service
sudo chown root:root $SERVICE_DESTINATION_PATH/jsk-spot-network.service
