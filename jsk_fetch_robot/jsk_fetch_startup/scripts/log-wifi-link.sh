#!/bin/bash

IFACE=wlan0
OUTPATH=/var/log/wifi.log
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

OUTDIR=$(dirname $OUTPATH)
if [ ! -d $OUTDIR ]; then
    mkdir -p $OUTDIR
fi

while true; do
    wifi_status >> $OUTPATH
    sleep $INTERVAL
done
