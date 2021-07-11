#!/bin/bash

PACKAGE_PATH=$(rospack find jsk_spot_startup)
SCRIPT_DESTINATION_PATH=/var/lib/robot
SERVICE_DESTINATION_PATH=/etc/systemd/system

if [ -e $SCRIPT_DESTINATION_PATH/update-network-connection.sh ]; then
    mv $PACKAGE_PATH/scripts/update-network-connection.sh $SCRIPT_DESTINATION_PATH/update-network-connection.sh
fi

if [ -e $SERVICE_DESTINATION_PATH/jsk_spot_networkd.service ]; then
    mv $PACKAGE_PATH/services/jsk_spot_networkd.service $SERVICE_DESTINATION_PATH/jsk_spot_networkd.service
fi
