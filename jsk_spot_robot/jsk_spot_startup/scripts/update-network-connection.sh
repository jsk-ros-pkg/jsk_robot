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

function connectWIFI() {
    sudo nmcli connection down sanshiro
    sudo nmcli connection up sanshiro
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

    # connect with Ethernet if available
    if [ $(existIF $IF_ETH) = 0 ]; then
        ping -c 1 -W 1 1.1.1.1 -I $IF_ETH
        if [ $? = 0 ]; then
            echo "ethernet is online"
            echo $CURRENT_CONNECTION_TYPE
            if [ $CURRENT_CONNECTION_TYPE = "ethernet" ]; then
                echo "connection type is still on ethernet"
            else
                echo "connection type switched to ethernet"
                CURRENT_CONNECTION_TYPE="ethernet"
                connectETH
            fi
            return
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
            echo "wifi is online"
            if [ $CURRENT_CONNECTION_TYPE = "wifi" ]; then
                echo "connection type is still on wifi"
            else
                echo "connection type switched to wifi"
                CURRENT_CONNECTION_TYPE="wifi"
                connectWIFI
            fi
            return
        else
            echo "wifi is not online now"
            sudo nmcli connection up sanshiro
            ping -c 1 -W 1 1.1.1.1 -I $IF_WIFI
            if [ $? = 0 ]; then
                echo "wifi is online"
                if [ $CURRENT_CONNECTION_TYPE = "wifi" ]; then
                    echo "connection type is still on wifi"
                else
                    echo "connection type switched to wifi"
                    CURRENT_CONNECTION_TYPE="wifi"
                    connectWIFI
                fi
                return
            else
                echo "wifi is still not online"
            fi
        fi
    else
        echo "wifi device is not found"
    fi

    # connect with LTE if available
    if [ $CURRENT_CONNECTION_TYPE = "lte" ]; then
        echo "connection type is still on lte"
    else
        echo "connection type switched to lte"
        CURRENT_CONNECTION_TYPE="lte"
        connectLTE
    fi
}

while :
do
    sleep 5
    updateConnection
done
