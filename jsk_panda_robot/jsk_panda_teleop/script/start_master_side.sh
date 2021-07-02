
function init_ros_ip () {
    export ROS_MASTER_URI=http://133.11.216.227:11311
    export ROS_IP=`hostname -I | cut -d' ' -f1`
}

function init_device() {
    echo "initializing device..."
    echo "giving access to /dev/tty0 and /dev/tty1"
    # TODO: add udev rule to remove sudo's
    sudo chmod 777 /dev/ttyACM0 && sudo chmod 777 /dev/ttyACM1
    # Run below if you use Omini for the first ime
    # rosrun omni_common initialize_device.sh 
}

function start_master_launch () {
    roslaunch jsk_panda_teleop start_panda_teleop_master_side.launch
}

init_ros_ip

init_device

start_master_launch
