#!/usr/bin/env bash

# Check storage percentage
STORAGE_PERCENTAGE=`df -h / | awk 'NR==2 {print $5}' | sed -e 's/\%//g'`

# If storage_percentage greater than 80%
if [ $STORAGE_PERCENTAGE -gt 80 ]; then
    SEND_EMAIL=1
    # In root directory
    ROOTDIR_STORAGE=`du -h -d 0 /* 2>/dev/null | sort -hr | awk '{print $2 " " $1}'`
    # In home directory
    HOMEDIR_STORAGE=`du -h -d 0 /home/* 2>/dev/null | sort -hr | awk '{print $2 " " $1}'`
fi

if [ -n "$SEND_EMAIL" ]; then
   rostopic pub -1 /email jsk_robot_startup/Email "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
subject: 'Storage warning'
body: 'Storage usage is ${STORAGE_PERCENTAGE}%. Please remove unnecessary files.


In root directory:

${ROOTDIR_STORAGE}


In home directory:

${HOMEDIR_STORAGE}'
sender_address: '$(hostname)@jsk.imi.i.u-tokyo.ac.jp'
receiver_address: 'fetch@jsk.imi.i.u-tokyo.ac.jp'
smtp_server: ''
smtp_port: ''
attached_files: []"
fi
