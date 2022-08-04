#!/bin/bash

if [ -e "arm64v8_User" ]; then
    if [ -e "arm64v8_User.tar.gz" ]; then
        echo "WARNING: Compressed arm64v8_User.tar.gz is found."
        read -p "WARNING: Are you sure to continue [y/N] ? " -n 1 -r
        echo    # (optional) move to a new line
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            tar -zcvf arm64v8_User.tar.gz arm64v8_User
        fi
    else
        tar -zcvf arm64v8_User.tar.gz arm64v8_User
    fi
fi

if [ -e "arm64v8_System" ]; then
    if [ -e "arm64v8_System.tar.gz" ]; then
        echo "WARNING: Compressed arm64v8_System.tar.gz is found."
        read -p "WARNING: Are you sure to continue [y/N] ? " -n 1 -r
        echo    # (optional) move to a new line
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            chmod 644 arm64v8_System/ros1_inst/share/pr2eus/*.l
            tar -zcvf arm64v8_System.tar.gz arm64v8_System
        fi
    else
        chmod 644 arm64v8_System/ros1_inst/share/pr2eus/*.l
        tar -zcvf arm64v8_System.tar.gz arm64v8_System
    fi
fi
