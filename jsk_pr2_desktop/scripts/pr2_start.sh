#!/bin/bash
set +e
PR2_NAME=$1
if [ "$1" == "" ]; then PR2_NAME="pr1012"; fi

AUSER=$(ssh applications@${PR2_NAME} "(PS1=applications source .bashrc &&  robot users --no-plist | grep 'Active User' | cut -d ' '  -f 3)")

if [ ! "${AUSER}" == "None"  ]; then
    read -p "${AUSER} is claiming PR2 !!! Would you like to use PR2? (y/n): "
    [ "$REPLY" == "y" -o  "$REPLY" == "Y" -o "$REPLY" == "yes" -o "$REPLY" == "YES" ] || exit 1;
fi

yes | ssh applications@${PR2_NAME} "(PS1=applications source .bashrc && robot claim)"
yes | ssh applications@${PR2_NAME} "(PS1=applications source .bashrc && robot stop)"
sleep 2
yes | ssh applications@${PR2_NAME} "(PS1=applications source .bashrc && robot start)"
sleep 5
