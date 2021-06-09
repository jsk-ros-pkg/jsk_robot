#!/usr/bin/env python
# -*- coding: utf-8 -*-
import linecache
import os
from subprocess import call
import sys
import syslog
import time

from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

log_file = '/var/log/syslog'
BASEDIR = os.path.abspath(os.path.dirname(log_file))


class ChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path != log_file:
            return
        num_lines = sum(1 for line in open(log_file))  # how many lines in the log file
        for i in range(5):  # check latest 5 lines
            target_line = linecache.getline(log_file, num_lines - i)
            if ('bluetoothd' in target_line) and ("Can't find device agent" in target_line):
                syslog.syslog(syslog.LOG_INFO, 'restart sixad service')
                call(['service', 'sixad', 'restart'])  # restart sixad
        linecache.clearcache()


if __name__ in '__main__':
    while True:
        event_handler = ChangeHandler()
        observer = Observer()
        observer.schedule(event_handler, BASEDIR, recursive=True)
        observer.start()
        try:
            while True:
                time.sleep(3)
        except KeyboardInterrupt:
            observer.stop()
        observer.join()
        time.sleep(1)
