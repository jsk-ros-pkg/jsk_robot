#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
import os
import sys
import errno
import subprocess
import signal
from threading import Thread
try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty
from roseus_remote.msg import RawCommand

ON_POSIX = 'posix' in sys.builtin_module_names
BUFF_SIZE = 1


class ROSEUSBridgeNode:
    def __init__(self):
        self.pub = rospy.Publisher('output', RawCommand)
        self.sub = rospy.Subscriber('input', RawCommand, self.raw_command_cb)
        rospy.on_shutdown(self._on_node_shutdown)
        while not rospy.is_shutdown():
            self.launch_roseus()
            self.main_loop()
            rospy.loginfo("respawning...")
            self.kill_roseus()

    def get_output(self, out, queue):
        while self.roseus_process.poll() is None:
            data = out.read(BUFF_SIZE)
            if data:
                queue.put(str(data))
        else:
            rospy.logerr("thread is dead")

    def launch_roseus(self):
        cmd = ['rosrun', 'roseus', 'roseus']
        self.roseus_process = subprocess.Popen(cmd,
                                               stdout=subprocess.PIPE,
                                               stderr=subprocess.PIPE,
                                               stdin=subprocess.PIPE,
                                               bufsize=BUFF_SIZE,
                                               close_fds=ON_POSIX,
                                               env=os.environ.copy(),
                                               preexec_fn=os.setpgrp)
        self.stdout_queue = Queue()
        self.stderr_queue = Queue()
        self.received_cmd = Queue()

        self.get_stdout_thread = Thread(target=self.get_output,
                                        args=(self.roseus_process.stdout,
                                              self.stdout_queue))
        self.get_stdout_thread.daemon = True
        self.get_stderr_thread = Thread(target=self.get_output,
                                        args=(self.roseus_process.stderr,
                                              self.stderr_queue))
        self.get_stderr_thread.daemon = True
        self.get_stdout_thread.start()
        self.get_stderr_thread.start()

    def kill_roseus(self):
        try:
            rospy.loginfo("send SIGTERM to roseus")
            self.roseus_process.terminate()
        except Exception as e:
            rospy.logwarn("escalated to kill")
            try:
                self.roseus_process.kill()
            except Exception as e:
                rospy.logerr('could not kill roseus: ' + str(e))

    def main_loop(self):
        while self.roseus_process.poll() is None:
            stdout = ""
            stderr = ""
            try:
                cmd = self.received_cmd.get_nowait()
                rospy.logdebug("write to stdin: " + cmd)
                self.roseus_process.stdin.write(cmd)
                self.roseus_process.stdin.flush()
            except IOError as e:
                if e.errno == errno.EINTR:
                    continue
            except Empty as e:
                pass
            except Exception as e:
                rospy.logwarn('error: ' + str(e))
            try:
                while not self.stdout_queue.empty():
                    stdout += self.stdout_queue.get_nowait()
            except Empty as e:
                pass
            try:
                while not self.stderr_queue.empty():
                    stderr += self.stderr_queue.get_nowait()
            except Empty as e:
                pass
            except Exception as e:
                rospy.logwarn('error: ' + str(e))
            if stdout != "":
                self.pub.publish(stdout.strip())
                rospy.logdebug("stdout: " + stdout)
            if stderr != "":
                self.pub.publish(stderr.strip())
                rospy.logdebug("stderr: " + stderr)
            rospy.sleep(0.1)
        else:
            rospy.logwarn("roseus process has been stopped.")

        if self.roseus_process.returncode != 0:
            rospy.logerr('roseus process exits abnormally with exit code: ' + str(self.roseus_process.returncode))

    def raw_command_cb(self, msg):
        cmd_str = str(msg.data).rstrip(' \t\r\n\0') + os.linesep
        self.received_cmd.put(cmd_str)

    def _on_node_shutdown(self):
        self.kill_roseus()

if __name__ == '__main__':
    rospy.init_node('roseus_bridge', anonymous=True)
    node = ROSEUSBridgeNode()
    rospy.spin()
