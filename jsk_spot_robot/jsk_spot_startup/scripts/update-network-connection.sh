#!/bin/bash

IF_ETH="enx70886b8f1b38"
IF_WIFI="wlxd037458e7f3c"
IF_LTE="enxf8b7975c750a"

CURRENT_CONNECTION_TYPE="none"

function existIF() {
    interface_name=$1
    for DEV in `find /sys/devices -name net | grep -v virtual`
    do
        if [ `ls $DEV/` = $interface_name ]; then
            echo 0
            return 0
        fi
    done
    echo 1
    return 1
}

function connectETH() {
    sudo ifmetric $IF_ETH 100
    sudo ifmetric $IF_LTE 101
    sudo ifmetric $IF_WIFI 101
}

function restartWIFI() {
    sudo nmcli connection down sanshiro
    sudo nmcli connection up sanshiro
}

function connectWIFI() {
    sudo ifmetric $IF_WIFI 100
    sudo ifmetric $IF_LTE 101
    sudo ifmetric $IF_ETH 102
}

function connectLTE() {
    sudo ifmetric $IF_LTE 100
    sudo ifmetric $IF_WIFI 101
    sudo ifmetric $IF_ETH 102
}

function checkConnection() {
    ping -c 1 -W 1.1.1.1
    if [ $? = 0 ]; then
        echo 0
        return 0
    else
        echo 1
        return 1
    fi
}

function updateConnection() {

    echo "Connection update"

    # Reconnect WIFI
    if [ $(existIF $IF_WIFI) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_WIFI
        if [ $? = 0 ]; then
            echo "wifi connected"
        else
            echo "wifi not connected. trying to connect sanshiro"
            restartWIFI
        fi
    else
        echo "wifi device not found. skipped wifi reconnection."
    fi

    # Check internet connection
    if [ $(checkConnection) = 0 ]; then
        echo "Network connected"
        return 0
    fi

    # connect with Ethernet if available
    if [ $(existIF $IF_ETH) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_ETH
        if [ $? = 0 ]; then
            echo "ethernet is online. switched to ethernet"
            connectETH
            return 0
        else
            echo "ethernet is not online"
        fi
    else
        echo "ethernet device is not found"
    fi

    # connect with Wi-Fi if available
    if [ $(existIF $IF_WIFI) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_WIFI
        if [ $? = 0 ]; then
            echo "wifi is online. switched to wifi"
            connectWIFI
            return 0
        else
            echo "wifi is not online now"
        fi
    else
        echo "wifi device is not found"
    fi

    # connect with LTE if available
    if [ $(existIF $IF_LTE) = 0 ]; then
        echo "connection type switched to lte"
        connectLTE
        return 1
    else
        echo "lte device is not found"
    fi

    echo "No network device found."
    return 1
}

while :
do
    sleep 1
    updateConnection
done
