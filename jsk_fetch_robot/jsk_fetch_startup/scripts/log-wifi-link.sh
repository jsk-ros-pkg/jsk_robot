#!/bin/bash

IFACE=wlan0
INTERVAL=10

wifi_status(){
    local ssid=$(iw $IFACE link | grep SSID | cut -d: -f2)
    local freq=$(iw $IFACE link | grep freq | cut -d':' -f2)
    local signal=$(cat /proc/net/wireless | grep $IFACE | awk '{print $3}')
    local level=$(cat /proc/net/wireless | grep $IFACE | awk '{print $4}')
    local noise=$(cat /proc/net/wireless | grep $IFACE | awk '{print $5}')
    local bitrate=$(iw $IFACE link | grep bitrate | cut -d':' -f2)
    echo "$(date -Iseconds),$ssid,$freq,$signal,$level,$noise,$bitrate"
}

print_help(){
    echo "Usage: $0 [-i INTERVAL (default: 10)] [-I IFACE (default: wlan0)]"
    exit 1
}

while getopts hi:I: o
do
    case "$o" in
        i)  INTERVAL=$OPTARG;;
        I)  IFACE="$OPTARG";;
        h)  print_help;;
    esac
done

while true; do
    wifi_status
    sleep $INTERVAL
done
