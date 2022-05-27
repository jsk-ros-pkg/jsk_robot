#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

IFS=':' read -r -a prefix_paths <<< "$CMAKE_PREFIX_PATH"
current_prefix_path="${prefix_paths[0]}"

set -x

cd $jsk_fetch_startup/supervisor_scripts
for file in $(ls *.conf); do
    sudo ln -s $jsk_fetch_startup/supervisor_scripts/$file /etc/supervisor/conf.d/$file
done
sudo supervisorctl reread
# Enable jsk_dstat job to save the csv log under /var/log
ln -s /home/fetch/Documents/jsk_dstat.csv /var/log/ros/jsk-dstat.csv

sudo ln -s $jsk_fetch_startup/config/config.bash /var/lib/robot/config.bash
sudo ln -s $jsk_fetch_startup/config/config.bash /var/lib/robot/config_outside.bash

set +x
