#!/bin/bash

IF_ETH=enxa0cec875af37
IF_WIFI=wlxc006c31b1a80
IF_LTE=enxf8b7975c750a
WIFI_PROFILE=sanshiro
CURRENT_CONNECTION=

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
    sudo ifmetric $IF_WIFI 101
    sudo ifmetric $IF_LTE 101
}

function restartWIFI() {
    sudo nmcli connection down $WIFI_PROFILE
    #sudo ip l set $IF_WIFI down
    #sudo ip l set $IF_WIFI up
    sudo nmcli connection up $WIFI_PROFILE
}

function connectWIFI() {
    sudo ifmetric $IF_WIFI 100
    sudo ifmetric $IF_ETH 102
    sudo ifmetric $IF_LTE 101
}

function connectLTE() {
    sudo ifmetric $IF_LTE 100
    sudo ifmetric $IF_WIFI 101
    sudo ifmetric $IF_ETH 102
}

function updateConnectionToETH() {
    if [ $(existIF $IF_ETH) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_ETH > /dev/null 2>&1
        if [ $? = 0 ]; then
            echo "Ethernet is online. switched to ethernet."
            CURRENT_CONNECTION="ETH"
            connectETH
        else
            echo "Ethernet is offline.: $?"
        fi
    else
        echo "No ethernet device is found."
    fi
}

function updateConnectionToWiFi() {
    if [ $(existIF $IF_WIFI) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_WIFI > /dev/null 2>&1
        if [ $? = 0 ]; then
            echo "Wifi is online. switched to wifi."
            CURRENT_CONNECTION="WiFi"
            connectWIFI
        else
            echo "Wifi is offline.: $?"
        fi
    else
        echo "No wifi device is found."
    fi
}

function updateConnectionToLTE() {
    if [ $(existIF $IF_LTE) = 0 ]; then
        echo "Connection type switched to lte"
        CURRENT_CONNECTION="LTE"
        connectLTE
    else
        echo "No lte device is found"
    fi

    echo "No network device found."
}

function updateConnection() {

    echo "Connection updating."

    # Reconnect WIFI
    if [ $(existIF $IF_WIFI) = 0 ]; then
        ping -c 1 -W 1 133.11.216.240 -I $IF_WIFI > /dev/null 2>&1
        if [ $? = 0 ]; then
            echo "wifi connected"
        else
            echo "wifi not connected. trying to connect $WIFI_PROFILE"
            restartWIFI
            updateConnectionToWiFi
        fi
    else
        echo "wifi device not found. skipped wifi reconnection."
    fi

    # Check internet connection
    ping -c 1 -W 1 1.1.1.1 > /dev/null 2>&1
    if [ $? = 0 ]; then
        echo "Network connected"
        return 0
    else
        echo "Network disconnected. Trying to reconnect."
    fi

    # First, try ethernet to connect with Ethernet if available
    updateConnectionToETH

    # Second, try wifi to connect with Wi-Fi if available
    updateConnectionToWiFi

    # connect with LTE if available
    updateConnectionToLTE
}

connectETH
CURRENT_CONNECTION="ETH"
while :
do
    sleep 5
    updateConnection
done
