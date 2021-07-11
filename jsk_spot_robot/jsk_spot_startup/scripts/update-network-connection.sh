#!/bin/bash

IF_ETH="hoge"
IF_WIFI="wlxd037458e7f3c"
IF_LTE="enxf8b7975c750a"

CURRENT_CONNECTION_TYPE=""

function connectETH() {
    sudo ifmetric $IF_ETH 100
    sudo ifmetric $IF_LTE 101
    sudo ifmetric $IF_WIFI 101↲
}

function connectWIFI() {↲
    sudo nmcli connection down sanshiro↲
    sudo nmcli connection up sanshiro↲
    sudo ifmetric $IF_WIFI 100↲
    sudo ifmetric $IF_LTE 101↲
    sudo ifmetric $IF_ETH 102
}↲
↲
function connectLTE() {↲
    sudo ifmetric $IF_LTE 100
    sudo ifmetric $IF_WIFI 101↲
    sudo ifmetric $IF_ETH 102
}↲

function updateConnection() {

    # connect with Ethernet if available
    ping -c 1 -W 1 1.1.1.1 -I $IF_ETH
    if [ $? = 0 ]; then
        if [ $CURRENT_CONNECTION_TYPE = "ethernet"]; then
            echo "connection type is still on ethernet"
        else
            echo "connection type switched to ethernet"
            CURRENT_CONNECTION_TYPE="ethernet"
            connectETH
        fi
        return
    fi

    # connect with Wi-Fi if available
    ping -c 1 -W 1 1.1.1.1 -I $IF_SANSHIRO
    if [ $? = 0 ]; then
        if [ $CURRENT_CONNECTION_TYPE = "wifi"]; then
            echo "connection type is still on wifi"
        else
            echo "connection type switched to wifi"
            CURRENT_CONNECTION_TYPE="wifi"
            connectWIFI
        fi
        return
    fi

    # connect with LTE if available
    if [ $CURRENT_CONNECTION_TYPE = "lte"]; then
        echo "connection type is still on lte"
    else
        echo "connection type switched to lte"
        CURRENT_CONNECTION_TYPE="lte"
        connectLTE
    fi
}

updateConnection
