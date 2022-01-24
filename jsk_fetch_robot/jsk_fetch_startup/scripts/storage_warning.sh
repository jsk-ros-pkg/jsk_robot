#!/usr/bin/env bash

MAIL_BODY=""

# Check storage percentage
STORAGE_PERCENTAGE=`df -h / | awk 'NR==2 {print $5}' | sed -e 's/\%//g'`

# If storage_percentage greater than 80%
if [ $STORAGE_PERCENTAGE -gt 80 ]; then
    MAIL_BODY=$MAIL_BODY"Storage usage is ${STORAGE_PERCENTAGE}. Please remove unnecessary files.\n"

    # In root directory
    ROOTDIR_STORAGE=`du -h -d 0 /* 2>/dev/null | sort -hr`
    MAIL_BODY=$MAIL_BODY"In root directory:\n"
    MAIL_BODY=$MAIL_BODY"${ROOTDIR_STORAGE}\n"
    
    # In home directory
    HOMEDIR_STORAGE=`du -h -d 0 /home/* 2>/dev/null | sort -hr`
    MAIL_BODY=$MAIL_BODY"In home directory:\n"
    MAIL_BODY=$MAIL_BODY"${HOMEDIR_STORAGE}\n"
fi

if [ -n "$MAIL_BODY" ]; then
   rostopic pub -1 /email jsk_robot_startup/Email "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
subject: 'Storage warning'
body: '$MAIL_BODY'
sender_address: '$(hostname)@jsk.imi.i.u-tokyo.ac.jp'
receiver_address: 'fetch@jsk.imi.i.u-tokyo.ac.jp'
smtp_server: ''
smtp_port: ''
attached_files: ''"
fi
