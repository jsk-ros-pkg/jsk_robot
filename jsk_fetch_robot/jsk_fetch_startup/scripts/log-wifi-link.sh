#!/bin/bash

IFACE=wlan0
INTERVAL=10
PING_COUNT=3
PING_TIMEOUT=1
PING_TARGET="www.google.com"

wifi_status(){
    local ssid=$(iw $IFACE link | grep SSID | cut -d: -f2)
    local freq=$(iw $IFACE link | grep freq | cut -d':' -f2)
    local signal=$(cat /proc/net/wireless | grep $IFACE | awk '{print $3}')
    local level=$(cat /proc/net/wireless | grep $IFACE | awk '{print $4}')
    local noise=$(cat /proc/net/wireless | grep $IFACE | awk '{print $5}')
    local bitrate=$(iw $IFACE link | grep bitrate | cut -d':' -f2)
    local ping_result_raw=$(ping -I $IFACE $PING_TARGET -c $PING_COUNT -W $PING_TIMEOUT | tail -n 1)
    if [ -z "$ping_result_raw" ]; then
        local ping_result_duration=999
    else
        local ping_result_duration=$(echo $ping_result_raw | cut -d " " -f 4 | cut -d "/" -f 2)
    fi
    echo "$(date -Iseconds),$ssid,$freq,$signal,$level,$noise,$bitrate, $ping_result_duration"
}

print_help(){
    echo "Usage: log-wifi-link.sh [-h]"
    echo "                        [-i INTERVAL (default: 10)]"
    echo "                        [-I IFACE (default: wlan0)]"
    echo "                        [-c PING_COUNT (default: 3)]"
    echo "                        [-W PING_TIMEOUT (default: 1)]"
    echo "                        [-t PING_TARGET (default: \"www.google.com\")]"
    exit 1
}

while getopts hi:I:c:W:t: o
do
    case "$o" in
        i)  INTERVAL=$OPTARG;;
        I)  IFACE="$OPTARG";;
        c)  PING_COUNT=$OPTARG;;
        W)  PING_TIMEOUT=$OPTARG;;
        t)  PING_TARGET="$OPTARG";;
        h)  print_help;;
    esac
done

while true; do
    wifi_status
    sleep $INTERVAL
done
