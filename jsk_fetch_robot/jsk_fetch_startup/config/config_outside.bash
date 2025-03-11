# This file is bash configuration for robot.conf
# This file must be at /var/lib/robot/config.bash

SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]:-$0}"; )" &> /dev/null && pwd 2> /dev/null; )";
. $SCRIPT_DIR/config.bash

if [ $(hostname) = 'fetch15' ]; then
  export NETWORK_DEFAULT_PROFILE_ID="sanshiro-outside";
elif [ $(hostname) = 'fetch1075' ]; then
  export NETWORK_DEFAULT_ROS_INTERFACE="fetch1075.local";
  export NETWORK_DEFAULT_PROFILE_ID="sanshiro-outside";
fi
