#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import subprocess
import os
import sys
import signal
import time
from threading import Thread
try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty

ON_POSIX = 'posix' in sys.builtin_module_names
BUFF_SIZE = 1

def get_output(out, queue):
    while p.poll() is None:
        data = out.read(BUFF_SIZE)
        if data:
            queue.put(data)
        else:
            print "nodata"
    else:
        print "error"


cmd = ['roseus']
p = subprocess.Popen(cmd,
                     stdout=subprocess.PIPE,
                     stderr=subprocess.PIPE,
                     stdin=subprocess.PIPE,
                     bufsize=BUFF_SIZE,
                     close_fds=ON_POSIX,
                     env=os.environ.copy(),
                     preexec_fn=os.setpgrp)

stdout_queue = Queue()
stderr_queue = Queue()

get_stdout_thread = Thread(target=get_output,
                            args=(p.stdout, stdout_queue))
get_stdout_thread.daemon = True
get_stderr_thread = Thread(target=get_output,
                            args=(p.stderr, stderr_queue))
get_stderr_thread.daemon = True
get_stdout_thread.start()
get_stderr_thread.start()

while p.poll() is None:
    print "alive?: ", p.poll()
    cmd = raw_input('cmd$ ')
    p.stdin.write(cmd)
    p.stdin.flush()
    stdout = ""
    stderr = ""
    try:
        while not stdout_queue.empty():
            stdout += stdout_queue.get_nowait()
    except:
        pass
    try:
        while not stderr_queue.empty():
            stderr += stderr_queue.get_nowait()
    except:
        pass
    print "stdout: ", stdout
    print "stderr: ", stderr
