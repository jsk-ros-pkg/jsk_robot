#!/bin/bash

IFACE=wlan0
INTERVAL=10

wifi_status(){
    local ssid=$(iw $IFACE link | grep SSID | cut -d: -f2)
    local freq=$(iw $IFACE link | grep freq | cut -d':' -f2)
    local signal=$(cat /proc/net/wireless | grep wlan0 | awk '{print $3}')
    local level=$(cat /proc/net/wireless | grep wlan0 | awk '{print $4}')
    local noise=$(cat /proc/net/wireless | grep wlan0 | awk '{print $5}')
    local bitrate=$(iw $IFACE link | grep bitrate | cut -d':' -f2)
    echo "$(date -Iseconds),$ssid,$freq,$signal,$level,$noise,$bitrate"
}

while true; do
    wifi_status >&1
    sleep $INTERVAL
done
