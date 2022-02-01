#!/bin/bash

PACKAGE_PATH=$(rospack find jsk_spot_startup)
SERVICE_SOURCE_PATH=$PACKAGE_PATH/services
SERVICE_DESTINATION_PATH=/etc/systemd/system

# install bash scripts
sudo cp $PACKAGE_PATH/scripts/roscore_poststart.bash /opt/ros/roscore_poststart.bash
sudo cp $PACKAGE_PATH/scripts/roscore_prestart.bash /opt/ros/roscore_prestart.bash
if [ ! -e /var/lib/robot/config.bash  ]; then
    sudo cp $PACKAGE_PATH/scripts/config.bash /var/lib/robot/config.bash
fi
sudo cp -f $PACKAGE_PATH/config/bash/jsk_profile.sh /etc/profile.d/jsk.sh

# install systemd unit files
cd $SERVICE_SOURCE_PATH
for service_file in $(ls ./*);
do
    sudo cp $service_file $SERVICE_DESTINATION_PATH/$service_file
    sudo chmod 644 $SERVICE_DESTINATION_PATH/$service_file
    sudo chown root:root $SERVICE_DESTINATION_PATH/$service_file
done
sudo systemctl daemon-reload
