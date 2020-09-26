#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2016-2017 Fetch Robotics Inc.
# Author(s): Angel Hernandez, Rushane Hua, Eric Relson, Aaron Blasdel

## @file network_monitor.py Script to make sure the network stays connected.

## Usage note: Script can be configured via environment variables:
##
## NETWORK_MONITOR_HOSTNAMES : comma delimited list of hostnames to use
## NETWORK_MONITOR_ENABLE_ARPING : 'true' causes arping to be used instead of ping
## NETWORK_MONITOR_ENABLE_REBOOT_OPTION : 'true' allows restarting robot (requires emailing capability)
## NETWORK_MONITOR_ENABLED : Set to 'false' to disable script

desc = ("Monitors network status and restart network manager when needed. "
        "Script is expected to be run as an upstart job. If run manually, "
        "you must either pass a hostname, or have NETWORK_MONITOR_HOSTNAMES defined.")

# Standard library
import argparse
import os
import time
import ConfigParser
from datetime import datetime
from subprocess import Popen, PIPE, call, check_output
from sys import exit
from time import sleep

import rospy
from sound_play.msg import SoundRequestActionGoal

# You must include network_monitor_emailer module for restarting to work.
try:
    import network_monitor_emailer
    if not network_monitor_emailer.credentials_exist():
        print "Note: Emailing disabled. Emailing requires auth process be completed"
        network_monitor_emailer = None
    else:
        print "Note: Emailing enabled."
except ImportError as e:
    network_monitor_emailer = None


VERSION = "0.5.2"

TOUCH_FILE = "/home/fetch/.Network_Monitor_Restarted_Robot"


def ping(use_arping, hosts):
    for host in hosts:
        for _ in xrange(5):
	    print datetime.now(),"Sending ping request"
            if use_arping:
                response = os.system("arping -I wlan0 -c 1 " + host)
            else:
                response = os.system("ping -c 1 -q " + host)
            if response == 0:
                return True
            sleep(5)
    return False

def get_sanshiro():
    return [['sanshiro', 'sanshiro']]

def get_saved_networks(filepaths=None):
    # NetworkManager has its own ID for networks, often multiple for the same
    # SSID, which it keeps in /etc/NetworkManager/system-connections/ as basic
    # config files. We parse these files for the ID and SSID.
    # Returns a list of (nmcli ID, SSID) tuples
    if not filepaths:
        default_dir = "/etc/NetworkManager/system-connections/"
        filepaths = os.listdir(default_dir)
        filepaths = [os.path.join(default_dir, fn) for fn in filepaths]
    networks = []
    for fn in filepaths:
        rc = ConfigParser.RawConfigParser()
        try:
            success = rc.read(fn)
            if success:
                networks.append([rc.get("connection", "id"),
                                 rc.get("802-11-wireless", "ssid")])
            else:
                print "Failed to parse " + fn
        except ConfigParser.Error as e:
            print "Failure with: " + fn
            print e

    return networks


# for reconnecting to 'sanshiro'
def reconnect_to_wlan():
    if call(["nmcli", "d", "disconnect", "iface", "wlan0"]) == 0:
        sleep(0.3)
        print "disconnect from wlan0"
    else:
        print "'nmcli d disconnect iface wlan0' do not work correctly!"

    if call(["nmcli", "c", "up", "id", "sanshiro"]) == 0:
        sleep(0.3)
        print "connect to sanshiro"
        return True
    else:
        print "'nmcli c up id sanshiro' do not work correctly!"
        return False


def get_avail_networks_by_strength(wifi_signal_output=None):
    # NetworkManager's nmcli lists networks with their signal strength (0-100).
    # Returns a list of ssids starting with the highest signal strength.
    if not wifi_signal_output:
        wifi_signal_output = check_output(["nmcli", "-f", "ssid,signal", "d", "wifi"]).splitlines()
        # reconnect to wifi until success
        while len(wifi_signal_output) == 1:
            reconnect_to_wlan()
            wifi_signal_output = check_output(["nmcli", "-f", "ssid,signal", "d", "wifi"]).splitlines()

    wifi_signals = {} # dict of {name: strength}
    for line in wifi_signal_output:
        if 'SIGNAL' in line or line == '':
            continue
        splitline = line.split("'")

        ssid = splitline[1]
        if not ssid:
            continue
        signal = int(splitline[2]) # get signal strength as int

        if ssid not in wifi_signals:
            wifi_signals[ssid] = signal
        elif signal > wifi_signals[ssid]:
            wifi_signals[ssid] = signal

    signal_sorted_wifi = [x[0] for x in reversed(sorted(wifi_signals.items()))]
    return signal_sorted_wifi


def connect_by_nmcli_id(nid, ssid="SSID not given"):
    # Tell nmcli to connect to it's saved connection 'nid'.
    # Returns True if successfully connected
    print "Trying to connect to '%s' (%s)" % (nid, ssid)
    if call(["nmcli", "c", "up", "id", nid, "--timeout", "10"]) == 0:
        print 'Connected'
        return True
    else:
        return False


def attempt_reconnect_to_network():
    # First, try to connect to sanshiro
    if connect_by_nmcli_id('sanshiro', 'sanshiro'):
        return True

    # Get list of stored 'wireless' connections; [id, ssid]
    known = get_saved_networks() # alternative : known = get_sanshiro()

    # Get list of unique available wifi network ssids and their strongest signal
    avail = get_avail_networks_by_strength()

    # Build the union of 'avail' and 'known', preserving the order of listing in avail
    to_try = []
    for av in avail:
        to_try.extend([kn for kn in known if kn[1] == av])

    # Attempt to connect to strongest wifi signal available
    for ntwk in to_try:
        if connect_by_nmcli_id(*ntwk):
            return True
    if reconnect_to_wlan():
        return True
    return False


def check_if_just_restarted(args, hosts):
    # Send an email and remove the TOUCH_FILE
    if network_monitor_emailer and os.path.isfile(TOUCH_FILE):
        # Check that network is up
        for _ in xrange(5):
            if ping(args.arping, hosts):
                break
        os.remove(TOUCH_FILE)
        print "Detected a robot restart by network monitor. Sending email..."
        network_monitor_emailer.main()


def main(args):
    # for fetch to speak wifi reconnection
    rospy.init_node("network_monitor")
    pub = rospy.Publisher('/robotsound_jp/goal', SoundRequestActionGoal, queue_size=1)
    sound_goal = SoundRequestActionGoal()
    sound_goal.goal_id.stamp = rospy.Time.now()
    sound_goal.goal.sound_request.sound = -3
    sound_goal.goal.sound_request.command = 1
    sound_goal.goal.sound_request.volume = 1.0
    sound_goal.goal.sound_request.arg2 = "jp"
    rospy.sleep(1) # wait for publisher to publish

    quit_after_restart_NM = False
    if args.hostname is None:
        # Check /etc/environment to see if a hostname can be found.
        # Otherwise assume fetchcore not setup yet; we don't want to
        # be restarting the wifi every 5 minutes in this case.
        if "NETWORK_MONITOR_HOSTNAMES" in os.environ:
            hostname = os.environ["NETWORK_MONITOR_HOSTNAMES"]
        else:
            print "No hostname to ping was defined. Sleeping monitor for 60s..."
            time.sleep(60)
            # Expect upstart job to rerun this script, at which point changes to
            # /etc/environment will be seen.
            exit()
        # Robustness: enabling this lets upstart job see changes to /etc/environment
        quit_after_restart_NM = True
    else:
        hostname = args.hostname

    hostnames = hostname.split(',')

    for num, hostname in enumerate(hostnames):
        if "://" in hostname: # remove *://
            hostnames[num] = hostname.split("://")[-1]
        if ":" in hostnames[num]: # remove port number
            hostnames[num] = hostnames[num].split(":")[0]

    if "NETWORK_MONITOR_ENABLE_ARPING" in os.environ:
        args.arping = os.environ["NETWORK_MONITOR_ENABLE_ARPING"].lower() == 'true'
    if "NETWORK_MONITOR_ENABLE_REBOOT_OPTION" in os.environ:
        args.enable_reboot_option = os.environ["NETWORK_MONITOR_ENABLE_REBOOT_OPTION"].lower() == 'true'
    if "NETWORK_MONITOR_ENABLED" in os.environ:
        if os.environ["NETWORK_MONITOR_ENABLED"].lower() == 'false':
            print("Network monitor has been disabled."
                  "To enable set `NETWORK_MONITOR_ENABLED=True` in /etc/environment.")
            sleep(15 * 60) # if disabled wait 15 minutes and restart to check again
            exit()

    print datetime.now(), "Ping and restart network Manager script Started!"
    print "Using host(s):", ', '.join(hostnames)

    # Sleep for 3 minutes to ensure on boot to ensure we don't reconnect before network inits
    print "Waiting 3 mins for network monitor to init"
    sleep(3 * 60)

    # Send email if TOUCH_FILE exists
    check_if_just_restarted(args, hostnames)

    while True:
        try:
            # Check if we can ping the access point
            if not ping(args.arping, hostnames):
                print datetime.now(), "Cannot connect to", ', '.join(hostnames)
                sound_goal.goal.sound_request.arg = "ピングが通りません"
                pub.publish(sound_goal)
                # Try to connect to any known network
                if not attempt_reconnect_to_network():
                    # TODO use check_output instead of Popen and handle truly
                    # bad issues with computer restarts or notification.
                    print datetime.now(), "Restarting network-manager"
                    sound_goal.goal.sound_request.arg = "ネットワークに再接続できないので、ネットワークマネージャを再起動します。"
                    pub.publish(sound_goal)
                    cmd = "sudo service network-manager restart"
                    proc = Popen(cmd, shell=True, stdout=PIPE)
                    print datetime.now(), "Sleeping monitor for 3 minutes"
                    sleep(3 * 60)
                    if args.enable_reboot_option:
                        # Check for severe failure; try restarting robot if email enabled
                        outp = check_output("nmcli d wifi | wc -l", shell=True)
                        try:
                            linecnt = int(outp)
                        except ValueError as e:
                            linecnt = 0
                        if linecnt == 1:
                            print "Output of 'nmcli d wifi' is empty, indicating " \
                                    "non-recoverable problem"
                            if network_monitor_emailer:
                                print datetime.now(), "Restarting robot as last resort"
                                # Touch our "robot has been restarted" file
                                open(TOUCH_FILE, 'a').close()
                                call("reboot".split())
                                # Alternate approach; Has issues; doesn't work
                                # first time, often, and just shuts off computer
                                #call("rtcwake -m off -s 20".split())
                                sleep(60) # computer will shut off during this time
                        else:
                            pass
                        # Regular failure
                        if quit_after_restart_NM:
                            # Expect upstart job to rerun this script, at which
                            # point changes to /etc/environment will be seen.
                            exit()

            sleep(5)
        except KeyboardInterrupt:
            break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument("hostname", nargs="?", type=str, default=None,
                        help="One or more comma delimited hostname(s) to ping. Hostnames will "
                        "be tried in order supplied until one is successfully contacted. Default: "
                        "Looks for NETWORK_MONITOR_HOSTNAMES in environment.")
    parser.add_argument("-a", "--arping", action='store_true',
                        help="Use arping instead of ping.")
    parser.add_argument("--enable-reboot-option", action='store_true',
                        help="Allow system to reboot the robot if output 'nmcli d wifi' is empty")
    parser.add_argument("-V", "--version", action='store_true',
                        help="Show network monitor version.")
    args = parser.parse_args()

    if args.version:
        print VERSION
        exit()

    # Ensure root
    if os.geteuid() != 0:
        print("ERROR: You need root permissions to run this script properly!")
        exit(1)

    main(args)
