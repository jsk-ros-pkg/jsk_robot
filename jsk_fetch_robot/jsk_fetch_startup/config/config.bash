# This file is bash configuration for robot.conf
# This file must be at /var/lib/robot/config.bash

if [ $(hostname) = 'fetch15' ]; then
  export DEFAULT_SPEAKER=13;
  export DEFAULT_ENGLISH_SPEAKER=cmu_us_bdl.flitevox;
  export DEFAULT_WARNING_SPEAKER=cmu_us_fem.flitevox;

  export USE_BASE_CAMERA_MOUNT=true;
  export USE_HEAD_BOX=true;
  export USE_HEAD_L515=true;
  export USE_INSTA360_STAND=true;

  export RS_SERIAL_NO_T265="925122110450";
  export RS_SERIAL_NO_D435_FRONTRIGHT="";
  export RS_SERIAL_NO_D435_FRONTLEFT="";
  export RS_SERIAL_NO_L515_HEAD="f0211890";

  export NETWORK_DEFAULT_WIFI_INTERFACE="wlan0";
  export NETWORK_DEFAULT_ROS_INTERFACE="133.11.216.217";
  export NETWORK_DEFAULT_PROFILE_ID="sanshiro-73B2";
elif [ $(hostname) = 'fetch1075' ]; then
  export DEFAULT_SPEAKER=2;
  export DEFAULT_ENGLISH_SPEAKER=cmu_us_slt.flitevox;
  export DEFAULT_WARNING_SPEAKER=cmu_us_lnh.flitevox;

  export USE_BASE_CAMERA_MOUNT=false;
  export USE_HEAD_BOX=false;
  export USE_HEAD_L515=true;
  export USE_INSTA360_STAND=true;

  export RS_SERIAL_NO_T265="";
  export RS_SERIAL_NO_D435_FRONTRIGHT="";
  export RS_SERIAL_NO_D435_FRONTLEFT="";
  export RS_SERIAL_NO_L515_HEAD="f0232270";

  export NETWORK_DEFAULT_WIFI_INTERFACE="wlan1";
  export NETWORK_DEFAULT_ROS_INTERFACE="133.11.216.218";
  export NETWORK_DEFAULT_PROFILE_ID="sanshiro-73B2";
fi
