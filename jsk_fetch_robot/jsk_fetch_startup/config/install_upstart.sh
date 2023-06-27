#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

IFS=':' read -r -a prefix_paths <<< "$CMAKE_PREFIX_PATH"
current_prefix_path="${prefix_paths[0]}"

set -x

cd $jsk_fetch_startup/upstart_scripts
for file in $(ls ./*.conf); do
    sudo cp $file /etc/init/
    sudo chown root:root /etc/init/$file
    sudo chmod 644 /etc/init/$file
    echo "copied $file to /etc/init"
done

set +x