# This script is originally copied from fetch_robots of fetchrobotics ( https://github.com/fetchrobotics/fetch_robots )
# This script is used by the roscore systemd service unit
echo "waiting for roscore to come up"
. /opt/ros/melodic/setup.sh
ret=`rosnode list`
while [ "$ret" = '' ]
do
    ret=`rosnode list`
    sleep 1;
done
