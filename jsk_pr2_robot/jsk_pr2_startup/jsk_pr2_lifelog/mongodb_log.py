#!/usr/bin/python

###########################################################################
#  mongodb_log.py - Python based ROS to MongoDB logger (multi-process)
#
#  Created: Sun Dec 05 19:45:51 2010
#  Copyright  2010-2012  Tim Niemueller [www.niemueller.de]
#             2010-2011  Carnegie Mellon University
#             2010       Intel Labs Pittsburgh
###########################################################################

#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Library General Public License for more details.
#
#  Read the full text in the LICENSE.GPL file in the doc directory.

# make sure we aren't using floor division
from __future__ import division, with_statement

PACKAGE_NAME='mongodb_log'
NODE_NAME='mongodb_log'
NODE_NAME_TEMPLATE='%smongodb_log'
WORKER_NODE_NAME = "%smongodb_log_worker_%d_%s"
QUEUE_MAXSIZE = 100

# import roslib; roslib.load_manifest(PACKAGE_NAME)
import rospy

# for msg_to_document
import mongodb_store.util

import os
import re
import sys
import time
import pprint
import string
import signal
import subprocess
from threading import Thread, Timer

from Queue import Empty
from optparse import OptionParser
from tempfile import mktemp
from datetime import datetime, timedelta
from time import sleep
from random import randint
from tf.msg import tfMessage
from sensor_msgs.msg import PointCloud, CompressedImage
from roslib.packages import find_node
#from rviz_intel.msg import TriangleMesh

use_setproctitle = True
try:
    from setproctitle import setproctitle
except ImportError:
    use_setproctitle = False


use_processes = False

# if use_processes:
from multiprocessing import Process, Lock, Condition, Queue, Value, current_process, Event
import multiprocessing as mp
# else:
    # from threading import Lock, Condition, Event
    # from Queue import Queue
    # def Value(t, val, lock=None):
        # return val


import genpy
import rosgraph.masterapi
import roslib.message
#from rospy import Time, Duration
import rostopic


from pymongo import SLOW_ONLY
from pymongo.errors import InvalidDocument, InvalidStringData

MongoClient = mongodb_store.util.import_MongoClient()

BACKLOG_WARN_LIMIT = 100
STATS_LOOPTIME     = 10
STATS_GRAPHTIME    = 60

class Counter(object):
    def __init__(self, value = None, lock = True):
        self.count = value or Value('i', 0, lock=lock)
        self.mutex = Lock()

    def increment(self, by = 1):
        with self.mutex: self.count.value += by

    def value(self):
        with self.mutex: return self.count.value

class Barrier(object):
    def __init__(self, num_threads):
        self.num_threads = num_threads
        self.threads_left = Value('i', num_threads, lock=True)
        self.mutex = Lock()
        self.waitcond = Condition(self.mutex)

    def wait(self):
        self.mutex.acquire()
        self.threads_left.value -= 1
        if self.threads_left.value == 0:
            self.threads_left.value = self.num_threads
            self.waitcond.notify_all()
            self.mutex.release()
        else:
            self.waitcond.wait()
            self.mutex.release()


class WorkerProcess(object):
    def __init__(self, idnum, topic, collname, in_counter_value, out_counter_value,
                 drop_counter_value, queue_maxsize,
                 mongodb_host, mongodb_port, mongodb_name, nodename_prefix):
        self.name = "WorkerProcess-%4d-%s" % (idnum, topic)
        self.id = idnum
        self.topic = topic
        self.collname = collname
        self.queue = Queue(queue_maxsize)
        self.out_counter = Counter(out_counter_value)
        self.in_counter  = Counter(in_counter_value)
        self.drop_counter = Counter(drop_counter_value)
        self.worker_out_counter = Counter()
        self.worker_in_counter  = Counter()
        self.worker_drop_counter = Counter()
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.mongodb_name = mongodb_name
        self.nodename_prefix = nodename_prefix
        self.quit = Value('i', 0)

        # print "Creating process %s" % self.name
        self.process = Process(name=self.name, target=self.run)
        # self.process = Thread(name=self.name, target=self.run)
        # print "created %s" % self.process
        self.process.start()
        # print "started %s" % self.process

    def init(self):
        global use_setproctitle
        if use_setproctitle:
            setproctitle("mongodb_log %s" % self.topic)

        self.mongoconn = MongoClient(self.mongodb_host, self.mongodb_port)
        self.mongodb = self.mongoconn[self.mongodb_name]
        self.mongodb.set_profiling_level = SLOW_ONLY

        self.collection = self.mongodb[self.collname]
        self.collection.count()

        self.queue.cancel_join_thread()

        # clear signal handlers in this child process, rospy will handle signals for us
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        worker_node_name = WORKER_NODE_NAME % (self.nodename_prefix, self.id, self.collname)
        # print "Calling init_node with %s from process %s" % (worker_node_name, mp.current_process())
        rospy.init_node(worker_node_name, anonymous=False)

        self.subscriber = None
        while not self.subscriber and not self.is_quit():
            try:
                msg_class, real_topic, msg_eval = rostopic.get_topic_class(self.topic, blocking=True)
                self.subscriber = rospy.Subscriber(real_topic, msg_class, self.enqueue, self.topic)
            except rostopic.ROSTopicIOException:
                print("FAILED to subscribe, will keep trying %s" % self.name)
                time.sleep(randint(1,10))
            except rospy.ROSInitException:
                print("FAILED to initialize, will keep trying %s" % self.name)
                time.sleep(randint(1,10))
                self.subscriber = None

    def run(self):
        self.init()

        print("ACTIVE: %s" % self.name)

        # run the thread
        self.dequeue()

        # free connection
        # self.mongoconn.end_request()

    def is_quit(self):
        return self.quit.value == 1

    def shutdown(self):
        if not self.is_quit():
            #print("SHUTDOWN %s qsize %d" % (self.name, self.queue.qsize()))
            self.quit.value = 1
            self.queue.put("shutdown")
            while not self.queue.empty(): sleep(0.1)
        #print("JOIN %s qsize %d" % (self.name, self.queue.qsize()))
        self.process.join()
        self.process.terminate()




    def qsize(self):
        return self.queue.qsize()

    def enqueue(self, data, topic, current_time=None):

        if not self.is_quit():
            if self.queue.full():
                try:
                    self.queue.get_nowait()
                    self.drop_counter.increment()
                    self.worker_drop_counter.increment()
                except Empty:
                    pass
            #self.queue.put((topic, data, current_time or datetime.now()))
            self.queue.put((topic, data, rospy.get_time(), data._connection_header))
            self.in_counter.increment()
            self.worker_in_counter.increment()

    def dequeue(self):
        while not self.is_quit():
            t = None
            try:
                t = self.queue.get(True)
            except IOError:
                # Anticipate Ctrl-C
                #print("Quit W1: %s" % self.name)
                self.quit.value = 1
                break
            if isinstance(t, tuple):
                self.out_counter.increment()
                self.worker_out_counter.increment()
                topic = t[0]
                msg   = t[1]
                ctime = t[2]
                connection_header = t[3]

                if isinstance(msg, rospy.Message):

                    try:
                        #print(self.sep + threading.current_thread().getName() + "@" + topic+": ")
                        #pprint.pprint(doc)
                        meta = {}
                        # switched to use inserted_at to match message_store
                        # meta["recorded"] = ctime or datetime.now()
                        meta["topic"]    = topic

                        if connection_header['latching'] == '1':
                            meta['latch'] = True
                        else:
                            meta['latch'] = False

                        if ctime is not None:
                            meta['inserted_at'] = datetime.utcfromtimestamp(ctime)
                        else:
                            meta['inserted_at'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())


                        mongodb_store.util.store_message(self.collection, msg, meta)

                    except InvalidDocument, e:
                        print("InvalidDocument " + current_process().name + "@" + topic +": \n")
                        print e
                    except InvalidStringData, e:
                        print("InvalidStringData " + current_process().name + "@" + topic +": \n")
                        print e

            else:
                #print("Quit W2: %s" % self.name)
                self.quit.value = 1

        # we must make sure to clear the queue before exiting,
        # or the parent thread might deadlock otherwise
        #print("Quit W3: %s" % self.name)
        self.subscriber.unregister()
        self.subscriber = None
        while not self.queue.empty():
            t = self.queue.get_nowait()
        print("STOPPED: %s" % self.name)


class SubprocessWorker(object):
    def __init__(self, idnum, topic, collname, in_counter_value, out_counter_value,
                 drop_counter_value, queue_maxsize,
                 mongodb_host, mongodb_port, mongodb_name, nodename_prefix, cpp_logger):

        self.name = "SubprocessWorker-%4d-%s" % (idnum, topic)
        self.id = idnum
        self.topic = topic
        self.collname = collname
        self.queue = Queue(queue_maxsize)
        self.out_counter = Counter(out_counter_value)
        self.in_counter  = Counter(in_counter_value)
        self.drop_counter = Counter(drop_counter_value)
        self.worker_out_counter = Counter()
        self.worker_in_counter  = Counter()
        self.worker_drop_counter = Counter()
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.mongodb_name = mongodb_name
        self.nodename_prefix = nodename_prefix
        self.quit = False
        self.qsize = 0

        self.thread = Thread(name=self.name, target=self.run)

        mongodb_host_port = "%s:%d" % (mongodb_host, mongodb_port)
        collection = "%s.%s" % (mongodb_name, collname)
        nodename = WORKER_NODE_NAME % (self.nodename_prefix, self.id, self.collname)

        self.process = subprocess.Popen([cpp_logger[0], "-t", topic, "-n", nodename,
                                         "-m", mongodb_host_port, "-c", collection],
                                        stdout=subprocess.PIPE)

        self.thread.start()

    def qsize(self):
        return self.qsize

    def run(self):
        while not self.quit:
            line = self.process.stdout.readline().rstrip()
            if line == "": continue
            arr = string.split(line, ":")
            self.in_counter.increment(int(arr[0]))
            self.out_counter.increment(int(arr[1]))
            self.drop_counter.increment(int(arr[2]))
            self.qsize = int(arr[3])

            self.worker_in_counter.increment(int(arr[0]))
            self.worker_out_counter.increment(int(arr[1]))
            self.worker_drop_counter.increment(int(arr[2]))

    def shutdown(self):
        self.quit = True
        self.process.kill()
        self.process.wait()


class MongoWriter(object):
    def __init__(self, topics = [], treat_as_regex=False,
                 all_topics = False, all_topics_interval = 5,
                 exclude_topics = [],
                 mongodb_host=None, mongodb_port=None, mongodb_name="roslog", mongodb_collection=None,
                 no_specific=False, nodename_prefix=""):
        self.all_topics = all_topics
        self.all_topics_interval = all_topics_interval
        self.exclude_topics = exclude_topics
        self.mongodb_host = mongodb_host
        self.mongodb_port = mongodb_port
        self.mongodb_name = mongodb_name
        self.mongodb_collection = mongodb_collection
        self.no_specific = no_specific
        self.nodename_prefix = nodename_prefix
        self.quit = False
        self.topics = set()
        self.collnames = set()
        #self.str_fn = roslib.message.strify_message
        self.sep = "\n" #'\033[2J\033[;H'
        self.in_counter = Counter()
        self.out_counter = Counter()
        self.drop_counter = Counter()
        self.workers = []

        global use_setproctitle
        if use_setproctitle:
            setproctitle("mongodb_log MAIN")

        self.exclude_regex = []
        for et in self.exclude_topics:
            self.exclude_regex.append(re.compile(et))
        self.exclude_already = []

        if treat_as_regex:
            topics = self.expand_regex_to_topics(topics)

        self.missing_topics = self.subscribe_topics(set(topics))
        self.fill_in_topics()


        if self.all_topics:
            print("All topics")
            self.ros_master = rosgraph.masterapi.Master(NODE_NAME_TEMPLATE % self.nodename_prefix)
            self.update_topics(restart=False)

        self.start_all_topics_timer()

    def expand_regex_to_topics(self, topics):
        expanded_topics = []
        published_topics = [t[0] for t in rospy.get_published_topics()]
        for pattern in topics:
            exp = re.compile(pattern)
            expanded_topics += filter(lambda t: exp.match(t) is not None, published_topics)
        return expanded_topics

    def subscribe_topics(self, topics):

        # print "existing topics %s" % self.topics

        # print "subscribing to topics %s" % topics

        missing_topics = set()
        for topic in topics:
            if topic and topic[-1] == '/':
                topic = topic[:-1]

            if topic in self.topics: continue
            if topic in self.exclude_already: continue

            do_continue = False
            for tre in self.exclude_regex:
                if tre.match(topic):
                    print("*** IGNORING topic %s due to exclusion rule" % topic)
                    do_continue = True
                    self.exclude_already.append(topic)
                    break
            if do_continue: continue

            # although the collections is not strictly necessary, since MongoDB could handle
            # pure topic names as collection names and we could then use mongodb[topic], we want
            # to have names that go easier with the query tools, even though there is the theoretical
            # possibility of name clashes (hence the check)
            if self.mongodb_collection:
                collname = self.mongodb_collection
            else:
                collname = mongodb_store.util.topic_name_to_collection_name(topic)
                if collname in self.collnames:
                    print("Two converted topic names clash: %s, ignoring topic %s"
                          % (collname, topic))
                    continue
            try:
                print("Adding topic %s" % topic)
                w = self.create_worker(len(self.workers), topic, collname)
                self.workers.append(w)
                self.collnames |= set([collname])
                self.topics |= set([topic])
            except Exception, e:
                print('Failed to subsribe to %s due to %s' % (topic, e))
                missing_topics.add(topic)

        return missing_topics


    def create_worker(self, idnum, topic, collname):
        try:
            msg_class, real_topic, msg_eval = rostopic.get_topic_class(topic, blocking=False)
        except Exception, e:
            print('Topic %s not announced, cannot get type: %s' % (topic, e))
            raise

        if real_topic is None:
            raise rostopic.ROSTopicException('topic type was empty, probably not announced')

        w = None
        node_path = None

        if not self.no_specific and msg_class == tfMessage:
            print("DETECTED transform topic %s, using fast C++ logger" % topic)
            node_path = find_node(PACKAGE_NAME, "mongodb_log_tf")
            if not node_path:
                print("FAILED to detect mongodb_log_tf, falling back to generic logger (did not build package?)")
        elif not self.no_specific and msg_class == PointCloud:
            print("DETECTED point cloud topic %s, using fast C++ logger" % topic)
            node_path = find_node(PACKAGE_NAME, "mongodb_log_pcl")
            if not node_path:
                print("FAILED to detect mongodb_log_pcl, falling back to generic logger (did not build package?)")
        elif not self.no_specific and msg_class == CompressedImage:
            print("DETECTED compressed image topic %s, using fast C++ logger" % topic)
            node_path = find_node(PACKAGE_NAME, "mongodb_log_cimg")
            if not node_path:
                print("FAILED to detect mongodb_log_cimg, falling back to generic logger (did not build package?)")
        """
        elif msg_class == TriangleMesh:
            print("DETECTED triangle mesh topic %s, using fast C++ logger" % topic)
            node_path = find_node(PACKAGE_NAME, "mongodb_log_trimesh")
            if not node_path:
                print("FAILED to detect mongodb_log_trimesh, falling back to generic logger (did not build package?)")
        """



        if node_path:
            w = SubprocessWorker(idnum, topic, collname,
                                 self.in_counter.count, self.out_counter.count,
                                 self.drop_counter.count, QUEUE_MAXSIZE,
                                 self.mongodb_host, self.mongodb_port, self.mongodb_name,
                                 self.nodename_prefix, node_path)

        if not w:
            print("GENERIC Python logger used for topic %s" % topic)
            w = WorkerProcess(idnum, topic, collname,
                              self.in_counter.count, self.out_counter.count,
                              self.drop_counter.count, QUEUE_MAXSIZE,
                              self.mongodb_host, self.mongodb_port, self.mongodb_name,
                              self.nodename_prefix)

        return w


    def run(self):
        looping_threshold = timedelta(0, STATS_LOOPTIME,  0)

        while not self.quit:
            started = datetime.now()

            # the following code makes sure we run once per STATS_LOOPTIME, taking
            # varying run-times and interrupted sleeps into account
            td = datetime.now() - started
            while not self.quit and td < looping_threshold:
                sleeptime = STATS_LOOPTIME - (td.microseconds + (td.seconds + td.days * 24 * 3600) * 10**6) / 10**6
                if sleeptime > 0: sleep(sleeptime)
                td = datetime.now() - started


    def shutdown(self):
        self.quit = True
        if hasattr(self, "all_topics_timer"): self.all_topics_timer.cancel()
        for w in self.workers:
            #print("Shutdown %s" % name)
            w.shutdown()


    def start_all_topics_timer(self):
        if not self.all_topics or self.quit: return
        self.all_topics_timer = Timer(self.all_topics_interval, self.update_topics)
        self.all_topics_timer.start()

    def start_fill_in_topics_timer(self):
        if len(self.missing_topics) == 0 or self.quit: return
        self.fill_in_topics_timer = Timer(self.all_topics_interval, self.fill_in_topics)
        self.fill_in_topics_timer.start()



    def update_topics(self, restart=True):
        """
        Called at a fixed interval (see start_all_topics_timer) to update the list of topics if we are logging all topics (e.g. --all-topics flag is given).
        """
        if not self.all_topics or self.quit: return
        ts = rospy.get_published_topics()
        topics = set([t for t, t_type in ts if t != "/rosout" and t != "/rosout_agg"])
        new_topics = topics - self.topics
        self.subscribe_topics(new_topics)
        if restart: self.start_all_topics_timer()

    def fill_in_topics(self, restart=True):
        """
        Called at a fixed interval (see start_all_topics_timer) to update the list of topics if we are logging all topics (e.g. --all-topics flag is given).
        """
        if len(self.missing_topics) == 0 or self.quit: return
        self.missing_topics = self.subscribe_topics(self.missing_topics)
        if restart: self.start_fill_in_topics_timer()


    def get_memory_usage_for_pid(self, pid):

        scale = {'kB': 1024, 'mB': 1024 * 1024,
                 'KB': 1024, 'MB': 1024 * 1024}
        try:
            f = open("/proc/%d/status" % pid)
            t = f.read()
            f.close()
        except:
            return (0, 0, 0)

        if t == "": return (0, 0, 0)

        try:
            tmp   = t[t.index("VmSize:"):].split(None, 3)
            size  = int(tmp[1]) * scale[tmp[2]]
            tmp   = t[t.index("VmRSS:"):].split(None, 3)
            rss   = int(tmp[1]) * scale[tmp[2]]
            tmp   = t[t.index("VmStk:"):].split(None, 3)
            stack = int(tmp[1]) * scale[tmp[2]]
            return (size, rss, stack)
        except ValueError:
            return (0, 0, 0)

    def get_memory_usage(self):
        size, rss, stack = 0, 0, 0
        for w in self.workers:
            pmem = self.get_memory_usage_for_pid(w.process.pid)
            size  += pmem[0]
            rss   += pmem[1]
            stack += pmem[2]
        #print("Size: %d  RSS: %s  Stack: %s" % (size, rss, stack))
        return (size, rss, stack)


def main(argv):
    parser = OptionParser()
    parser.usage += " [TOPICs...]"
    parser.add_option("--nodename-prefix", dest="nodename_prefix",
                      help="Prefix for worker node names", metavar="ROS_NODE_NAME",
                      default="")
    parser.add_option("--mongodb-host", dest="mongodb_host",
                      help="Hostname of MongoDB", metavar="HOST",
                      default=rospy.get_param("mongodb_host", "localhost"))
    parser.add_option("--mongodb-port", dest="mongodb_port",
                      help="Hostname of MongoDB", type="int",
                      metavar="PORT", default=rospy.get_param("mongodb_port", 27017))
    parser.add_option("--mongodb-name", dest="mongodb_name",
                      help="Name of DB in which to store values",
                      metavar="NAME", default=rospy.get_param("robot/database"))
    parser.add_option("--mongodb-collection", dest="mongodb_collection",
                      help="Name of Collection in which to store values. All topics are stored in the collection if used this option, otherwise topic names are used as collections",
                      metavar="COLLECTION", default=rospy.get_param("robot/name"))
    parser.add_option("-a", "--all-topics", dest="all_topics", default=False,
                      action="store_true",
                      help="Log all existing topics (still excludes /rosout, /rosout_agg)")
    parser.add_option("-e", "--regex", dest="treat_as_regex", default=False,
                      help="Log topics matching the follow regular expression",
                      action="store_true")
    parser.add_option("--all-topics-interval", dest="all_topics_interval", default=5,
                      help="Time in seconds between checks for new topics", type="int")
    parser.add_option("-x", "--exclude", dest="exclude",
                      help="Exclude topics matching REGEX, may be given multiple times",
                      action="append", type="string", metavar="REGEX", default=[])
    parser.add_option("--no-specific", dest="no_specific", default=False,
                      action="store_true", help="Disable specific loggers")

    (options, args) = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    if not options.all_topics and len(args) == 0:
        parser.print_help()
        return

    try:
        rosgraph.masterapi.Master(NODE_NAME_TEMPLATE % options.nodename_prefix).getPid()
    except socket.error:
        print("Failed to communicate with master")

    mongowriter = MongoWriter(topics=args,
                              treat_as_regex=options.treat_as_regex,
                              all_topics=options.all_topics,
                              all_topics_interval = options.all_topics_interval,
                              exclude_topics = options.exclude,
                              mongodb_host=options.mongodb_host,
                              mongodb_port=options.mongodb_port,
                              mongodb_name=options.mongodb_name,
                              mongodb_collection=options.mongodb_collection,
                              no_specific=options.no_specific,
                              nodename_prefix=options.nodename_prefix)

    def signal_handler(signal, frame):
        mongowriter.shutdown()

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    mongowriter.run()

if __name__ == "__main__":
    main(sys.argv)
