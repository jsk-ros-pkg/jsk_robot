#!/bin/bash

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

function updateConnection() {

    echo "Connection updating."

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
    ping -c 1 -W 1 1.1.1.1
    if [ $? = 0 ]; then
        echo "Network connected"
        return 0
    else
        echo "Network disconnected. Trying to reconnect."
    fi

    # connect with Ethernet if available
    if [ $(existIF $IF_ETH) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_ETH
        if [ $? = 0 ]; then
            echo "Ethernet is online. switched to ethernet."
            connectETH
            return 0
        else
            echo "Ethernet is offline."
        fi
    else
        echo "No ethernet device is found."
    fi

    # connect with Wi-Fi if available
    if [ $(existIF $IF_WIFI) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_WIFI
        if [ $? = 0 ]; then
            echo "Wifi is online. switched to wifi."
            connectWIFI
            return 0
        else
            echo "Wifi is offline."
        fi
    else
        echo "No wifi device is found."
    fi

    # connect with LTE if available
    if [ $(existIF $IF_LTE) = 0 ]; then
        echo "Connection type switched to lte"
        connectLTE
        return 1
    else
        echo "No lte device is found"
    fi

    echo "No network device found."
    return 1
}

while :
do
    sleep 1
    updateConnection
done
