#!/bin/bash

echo ""
echo "This scripts copies udev rules for dual hands to /etc/udev/rules.d"
echo ""

sudo cp `rospack find detachable_tools`/udev/90-dual-hands.rules /etc/udev/rules.d

echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
