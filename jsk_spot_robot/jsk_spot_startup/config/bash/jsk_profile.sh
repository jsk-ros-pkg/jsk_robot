if [ -e /var/lib/robot/config.bash ]; then
    source /var/lib/robot/config.bash
else
    echo "Cannot find /var/lib/robot/config.bash file. Please make one at first."
fi
