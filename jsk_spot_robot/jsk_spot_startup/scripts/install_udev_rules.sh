#!/bin/bash

sudo cp $(rospack find jsk_spot_startup)/udev_rules/* /etc/udev/rules.d/
sudo udevadm control --reload-rules
