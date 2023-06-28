#!/usr/bin/env python
# -*- coding:utf-8 -*-

import subprocess
import re

import requests
import rospy
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


cell_number_re = re.compile(r"^Cell\s+(?P<cellnumber>.+)\s+-\s+Address:\s(?P<mac>.+)$")
regexps = [
    re.compile(r"^ESSID:\"(?P<essid>.*)\"$"),
    re.compile(r"^Protocol:(?P<protocol>.+)$"),
    re.compile(r"^Mode:(?P<mode>.+)$"),
    re.compile(r"^Frequency:(?P<frequency>[\d.]+) (?P<frequency_units>.+) \(Channel (?P<channel>\d+)\)$"),
    re.compile(r"^Encryption key:(?P<encryption>.+)$"),
    re.compile(r"^Quality=(?P<signal_quality>\d+)/(?P<signal_total>\d+)\s+Signal level=(?P<signal_level_dBm>.+) d.+$"),
    re.compile(r"^Signal level=(?P<signal_quality>\d+)/(?P<signal_total>\d+).*$"),
]

# Detect encryption type
wpa_re = re.compile(r"IE:\ WPA\ Version\ 1$")
wpa2_re = re.compile(r"IE:\ IEEE\ 802\.11i/WPA2\ Version\ 1$")


def iwlist_scan(interface='wlan0'):
    """Runs the comnmand to scan the list of networks.

    Must run as super user.
    Does not specify a particular device, so will scan all network devices.
    """
    cmd = ["iwlist", interface, "scan"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    points = proc.stdout.read().decode('utf-8')
    return points


def parse_iwlist(content):
    """Parses the response from the command "iwlist scan"

    """
    cells = []
    lines = content.split('\n')
    for line in lines:
        line = line.strip()
        cellNumber = cell_number_re.search(line)
        if cellNumber is not None:
            cells.append(cellNumber.groupdict())
            continue
        wpa = wpa_re.search(line)
        if wpa is not None:
            cells[-1].update({'encryption': 'wpa'})
        wpa2 = wpa2_re.search(line)
        if wpa2 is not None:
            cells[-1].update({'encryption': 'wpa2'})
        for expression in regexps:
            result = expression.search(line)
            if result is not None:
                if 'encryption' in result.groupdict():
                    if result.groupdict()['encryption'] == 'on':
                        cells[-1].update({'encryption': 'wep'})
                    else:
                        cells[-1].update({'encryption': 'off'})
                else:
                    cells[-1].update(result.groupdict())
                continue
    return cells


class LocationNode(object):

    def __init__(self):
        self.network_interface = rospy.get_param('~network_interface', 'wlan0')
        self.location_key = rospy.get_param('~location_key')
        self.service = rospy.Service('~get_location',
                                     Trigger, self.get_location)

    def get_location(self, req):
        aps = parse_iwlist(iwlist_scan(self.network_interface))
        contents = [{"macAddress": str(ap['mac']), "age": 0}
                    for ap in aps]
        headers = {'Content-type': 'application/json'}
        params = {'key': self.location_key,
                  'language': 'ja'}
        data = '{"wifiAccessPoints": ' + '[{}]'.format(contents) + '}'

        response = requests.post('https://www.googleapis.com/geolocation/v1/geolocate',
                                 params=params, headers=headers, data=data)
        # {'location': {'lat': 35.7145647, 'lng': 139.766433}, 'accuracy': 19.612}
        hoge = response.json()

        lat = hoge['location']['lat']
        lng = hoge['location']['lng']
        # lat = 35.715106109567415
        # lng = 139.77380123496505
        response = requests.get(
            'https://maps.googleapis.com/maps/api/geocode/json?latlng={},{}'.format(lat, lng),
            headers=headers, params=params)
        return TriggerResponse(success=True,
                               message=response.text)


if __name__ == '__main__':
    rospy.init_node('location_node')
    location_node = LocationNode()
    rospy.spin()
