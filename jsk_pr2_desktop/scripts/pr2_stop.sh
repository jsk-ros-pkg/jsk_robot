#!/bin/bash
set +e

AUSER=$(ssh applications@pr1012 "(PS1=applications source .bashrc &&  robot users --no-plist | grep 'Active User' | cut -d ' '  -f 3)")
if [ ! "${AUSER}" == "None"  ]; then
    read -p "${AUSER} is claiming PR2! Would you like to stop PR2? (y/n): "
    [ "$REPLY" == "y" -o  "$REPLY" == "Y" -o "$REPLY" == "yes" -o "$REPLY" == "YES" ] || exit 1;
fi

yes | ssh applications@pr1012 "(PS1=applications source .bashrc && robot stop)"
yes | ssh applications@pr1012 "(PS1=applications source .bashrc && robot release)"
# pkill -9 rviz
