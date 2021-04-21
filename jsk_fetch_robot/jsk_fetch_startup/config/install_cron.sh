#!/bin/bash

jsk_fetch_startup=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`"/.. > /dev/null && pwd)

IFS=':' read -r -a prefix_paths <<< "$CMAKE_PREFIX_PATH"
current_prefix_path="${prefix_paths[0]}"

set -x

cd $jsk_fetch_startup/cron_scripts
sudo -u fetch crontab cron_fetch.conf
echo "Set cron jobs for fetch user"
sudo -u root crontab cron_root.conf
echo "Set cron jobs for root user"
set +x
