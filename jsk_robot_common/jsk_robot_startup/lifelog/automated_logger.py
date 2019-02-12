#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from __future__ import print_function
from collections import defaultdict
import copy
import click
import itertools
import pymongo
import re
import rosgraph.masterapi
import rospy
import rosservice
import socket
import subprocess as sp
import sys
import time
import traceback
import uuid
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from nodelet.srv import NodeletLoad, NodeletUnload, NodeletList


KEYDELIM = '|'
TOPICPREFIX = 'topic:'


def match(regex, s):
    for r in regex:
        if re.match(r, s):
            return True
    return False


def entopic(name):
    return TOPICPREFIX + name


def detopic(name):
    if name.startswith(TOPICPREFIX):
        name = name[len(TOPICPREFIX):]
    return name


class Edge(object):
    def __init__(self, start, end, label=None):
        super(Edge, self).__init__()

        if not start or not end:
            raise ValueError('invalid graph {}->{}'.format(start, end))
        self.start = start
        self.end = end
        if label is None:
            label = str()
        elif not isinstance(label, str):
            raise ValueError('invalid label {}'.format(label))
        self.label = label
        self.key = '{}{}{}'.format(self.start, KEYDELIM, self.end)

    def __eq__(self, e):
        return self.key == e.key

    def __ne__(self, e):
        return not self.__eq__(e)

    def __str__(self):
        return '{}->{}'.format(self.start, self.end)


class EdgeList(object):
    def __init__(self):
        super(EdgeList, self).__init__()

        self.edges = dict()

    @property
    def ranks(self):
        base = self.edges.values()[0][0]
        ranks = {base.start: 0, base.end: 1}
        q = set([base.start, base.end])
        while q:
            n = q.pop()
            if n in ranks:
                continue
            rank = ranks[n]
            edges = self.find_edge_by_node(n, upward=True, downward=True)
            for edge in edges:
                if edge.start == n:
                    ranks[edge.end] = rank + 1
                    q.add(edge.end)
                elif edge.end == n:
                    ranks[edge.start] = rank - 1
                    q.add(edge.start)
                else:
                    raise RuntimeError(
                        'invalid edge {} for finding node {}'.format(edge, n))
        min_rank = min(ranks.values())
        for k in ranks:
            ranks[k] += -min_rank
        return ranks

    def __iter__(self):
        return itertools.chain(*[v for v in self.edges.values()])

    def __contains__(self, edge):
        return edge.key in self.edges

    def add(self, edge):
        key = edge.key
        if key in self.edges:
            edge_list = self.edges[key]
            if edge in edge_list:
                label_list = [e.label for e in edge_list]
                if edge.label not in label_list:
                    edge_list.append(edge)
                    return True
            else:
                return False
        else:
            self.edges[key] = [edge]
            return True

    def add_edge(self, start, end, label=None):
        return self.add(Edge(start, end, label))

    def add_edges(self, start, end, direction, label=None):
        updated = False
        if direction in ['o', 'b']:
            updated = self.add_edge(start, end, label) or updated
        if direction in ['i', 'b']:
            updated = self.add_edge(end, start, label) or updated
        return updated

    def delete(self, edge):
        key = edge.key
        if key in self.edges:
            l = self.edges[key]
            if edge in l:
                l.remove(edge)
                return True

    def delete_by_node(self, node):
        edge_list = [v for k, v in self.edges.items()\
                     if k.startswith(node + KEYDELIM) or k.endswith(KEYDELIM + node)]
        for l in edge_list:
            for edge in l:
                self.delete(edge)

    def find_edge_by_node(self, node, upward=True, downward=True):
        edges = []
        if upward:
            edges += [v for k, v in self.edges.items() if k.endswith(KEYDELIM + node)]
        if downward:
            edges += [v for k, v in self.edges.items() if k.startswith(node + KEYDELIM)]
        return itertools.chain(*edges)


class Graph(object):
    def __init__(self, base_node):
        super(Graph, self).__init__()

        self.base_node = base_node

        self.nodes = set()
        self.topics = set()
        self.srvs = set()

        self.blacklist_nodes = set(['^/rosout', '.*lifelog.*', '/realtime_loop', '/move_base_node'])
        self.blacklist_topics = set([
            '.*/bond$',  # nodelets
            '^/tf', '^/tf_static',  # tfs
            '.*/depth/.*', '.*compressed.*', '.*theora.*', '.*image_color$',  # images
            '/lifelog.*$',  # lifelog topics
            '.*/parameter_updates', '.*/parameter_descriptions',  # dynamic reconfigure
            '.*/result', '.*/goal', '.*/status$', '.*/cancel$', '.*/feedback$',  # actionlib
        ])
        self.bad_nodes = {}
        self.node_node_conn = EdgeList()
        self.node_topic_conn = EdgeList()
        self.node_srv_conn = EdgeList()

        name = rospy.get_name()
        if name == '/unnamed':
            name = '/automated_logger'
        self.this_name = name
        self.master = rosgraph.masterapi.Master(self.this_name)
        self.node_uri_cache = dict()
        self.uri_node_cache = dict()

    def add_blacklist_topic(self, topic):
        self.blacklist_topics.add(topic)

    def add_blacklist_node(self, node):
        self.blacklist_nodes.add(node)

    def update(self, upward=True, downward=True):
        updated = self.update_nodes(upward=upward, downward=downward)
        return self.update_conns() or updated

    def update_nodes(self, upward=True, downward=True):
        try:
            res = self.master.getSystemState()
        except rosgraph.masterapi.MasterException as e:
            rospy.logerr('Unable to contact master', str(e), traceback.format_exc())
            return False

        node_topics = defaultdict(lambda: defaultdict(set))
        topic_nodes = defaultdict(lambda: defaultdict(set))
        pubs, subs, srvs = res
        for topic, nodes in pubs:
            for node in nodes:
                node_topics[node]["pub"].add(topic)
            topic_nodes[topic]["pub"] = nodes
        for topic, nodes in subs:
            for node in nodes:
                node_topics[node]["sub"].add(topic)
            topic_nodes[topic]["sub"] = nodes

        if self.base_node not in node_topics:
            raise ValueError('base_node {} not found'.format(self.base_node))

        node_set = set()
        topic_set = set()
        srv_set = set()

        if downward:
            q = set([self.base_node])
            while q:
                n = q.pop()
                for t in node_topics[n]["pub"]:
                    if match(self.blacklist_topics, t):
                        continue
                    added = False
                    for nn in topic_nodes[t]["sub"]:
                        if match(self.blacklist_nodes, nn):
                            continue
                        if nn not in node_set:
                            q.add(nn)
                        node_set.add(nn)
                        added = True
                    if added:
                        topic_set.add(t)

        if upward:
            q = set([self.base_node])
            while q:
                n = q.pop()
                for t in node_topics[n]["sub"]:
                    if match(self.blacklist_topics, t):
                        continue
                    added = False
                    for nn in topic_nodes[t]["pub"]:
                        if match(self.blacklist_nodes, nn):
                            continue
                        if nn not in node_set:
                            q.add(nn)
                        node_set.add(nn)
                        added = True
                    if added:
                        topic_set.add(t)

        node_set.add(self.base_node)

        node_srv_conn = EdgeList()
        for info in srvs:
            node, srv = info[1][0], info[0]
            if node in node_set:
                srv_set.add(srv)
                node_srv_conn.add_edge(node, srv)

        self.nodes = node_set
        self.topics = topic_set
        self.srvs = srv_set
        self.node_srv_conn = node_srv_conn

        return True

    def get_node_uri(self, node):
        try:
            uri = self.master.lookupNode(node)
            self.node_uri_cache[node] = uri
            self.uri_node_cache[uri] = node
            return uri
        except Exception as e:
            rospy.logerr('Failed to lookup node', str(e), traceback.format_exc())
            if node in self.node_uri_cache:
                return self.node_uri_cache[node]
            else:
                raise e

    def update_conns(self):
        updated = False

        for node in self.nodes:
            try:
                uri = self.get_node_uri(node)
                api = ServerProxy(uri)
            except:
                continue

            timeout = socket.getdefaulttimeout()
            try:
                if node in self.bad_nodes:
                    socket.setdefaulttimeout(0.2)
                else:
                    socket.setdefaulttimeout(1.0)
                code, msg, businfo = api.getBusInfo(self.this_name)
            except Exception as e:
                self.bad_nodes[node] = e
                code = -1
                msg = traceback.format_exc()
            finally:
                socket.setdefaulttimeout(timeout)

            if code != 1:
                rospy.logerr('Could not get bus info of node {}: {}'.format(
                    node, msg))
                continue

            for info in businfo:
                if len(info) < 5: continue

                conn_id, dest_id, direction, transport, topic = info[:5]
                if len(info) > 5:
                    connected = info[5]
                else:
                    connected = True


                if connected:
                    if dest_id.startswith('http://'):
                        dest_id = self.uri_node_cache.get(dest_id, None)
                    if dest_id in self.nodes and\
                       not match(self.blacklist_nodes, dest_id) and\
                       not match(self.blacklist_topics, topic):
                        self.topics.add(topic)
                        updated = self.node_topic_conn.add_edges(
                            node, entopic(topic), direction) or updated
                        updated = self.node_node_conn.add_edges(
                            node, dest_id, direction, topic) or updated

        for n, err in self.bad_nodes.items():
            rospy.logerr('bad node {}: {}'.format(n, err))

        return updated

    def to_graph(self, out, view=False):
        import os
        from graphviz import Digraph

        g = Digraph()
        g.attr("graph", rankdir="LR")
        for edges in self.node_node_conn.edges.values():
            start, end = edges[0].start, edges[0].end
            label = ''.join([e.label + '\l' for e in edges])
            size = len(edges)
            g.edge(start, end, label=label, penwidth=str(size))
        g.node(self.base_node, peripheries="2")

        # compute ranks
        ranks = self.node_node_conn.ranks
        rank_max = max(ranks.values())
        for i in range(rank_max):
            nodes = [k for k, v in ranks.items() if v == i]
            body = '{rank=same; '
            for n in nodes:
                body += '"' + n + '"; '
            body += '}'
            g.body.append(body)

        save_dir = os.path.dirname(os.path.abspath(out))
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        g.render(out, view=view)
        return True


class NodeletManager(object):
    _managers = list()

    def __init__(self, name, color):
        super(NodeletManager, self).__init__()
        self.name = name
        self.manager = None
        if color is None:
            color = 'white'
        self.color = color

        self._list_nodelet_timeout = rospy.Duration(5.0)
        self._last_list_nodelet_stamp = rospy.Time(0)
        self._nodelets = set()
        self._loaded_nodelets = set()
        self._srv_wait_timeout = 0.3

        self._list_srv = rospy.ServiceProxy(
            self.name + "/list", NodeletList)
        self._load_srv = rospy.ServiceProxy(
            self.name + "/load_nodelet", NodeletLoad)
        self._unload_srv = rospy.ServiceProxy(
            self.name + "/unload_nodelet", NodeletUnload)

        try:
            self._list_srv.wait_for_service(timeout=self._srv_wait_timeout)
            self.loginfo("Found existing manager")
        except rospy.exceptions.ROSException:
            self.start_manager()

    @classmethod
    def get_manager(cls, name):
        colors = ['green', 'yellow', 'blue', 'magenta', 'cyan', 'white',
                  'bright_black', 'bright_green', 'bright_yellow',
                  'bright_blue', 'bright_magenta', 'bright_cyan', 'bright_white']
        for m in cls._managers:
            if m.name == name:
                return m
        index = len(cls._managers) % len(colors)
        m = cls(name, colors[index])
        cls._managers.append(m)
        return m

    def logfmt(self, func, msg):
        func(click.style('[{}] {}'.format(self.name, msg), fg=self.color))

    def loginfo(self, msg):
        self.logfmt(rospy.loginfo, msg)

    def logwarn(self, msg):
        self.logfmt(rospy.logwarn, msg)

    def logerr(self, msg):
        self.logfmt(rospy.logerr, msg)

    def start_manager(self):
        if self.manager is None:
            name = self.name
            if name.startswith('/'):
                name = name[1:]
            self.manager = sp.Popen(
                ["rosrun", "nodelet", "nodelet", "manager",
                 "__name:={}".format(name)],)
            self.manager.daemon = True
            self.loginfo('started manager')

        try:
            self._list_srv.wait_for_service(timeout=5.0)
            return True
        except rospy.exceptions.ROSException:
            self.logerr('Failed to start manager')
            self.logerr(traceback.format_exc())
            return False

    def stop_manager(self):
        self.unload_all()
        if self.manager is not None and\
           self.manager.poll() is not None:
            start = time.time()
            timeout = 10.0
            self.manager.terminate()
            while self.manager.poll() is None:
                time.sleep(0.1)
                elapsed = time.time() - start
                if elapsed > timeout:
                    self.manager.kill()
                    self.manager.wait()
                    break
            retcode = self.manager.poll()
            if retcode == 0:
                self.loginfo('stoped manager')
            else:
                self.logerr('Failed to stop manager (code: {})'.format(retcode))
                return False
        return True

    @property
    def nodelets(self):
        now = rospy.Time.now()
        if rospy.Time.now() - self._last_list_nodelet_stamp > self._list_nodelet_timeout:
            try:
                self._list_srv.wait_for_service(timeout=self._srv_wait_timeout)
                self._nodelets = set(self._list_srv().nodelets)
            except Exception as e:
                self.logerr("Failed to list nodelet")
                self._nodelets = set()
        return self._nodelets

    @property
    def loaded_nodelets(self):
        nodelets = self.nodelets
        if nodelets:
            self._loaded_nodelets = nodelets & self._loaded_nodelets
        return self._loaded_nodelets

    def load(self, name, type, remappings=None, my_argv=None, params=None):
        if name in self.loaded_nodelets:
            self.logwarn('{} already loaded'.format(name))
            return True

        source, target = [], []
        if remappings:
            source = remappings.keys()
            target = [remappings[k] for k in source]
            # resolve private namespace
            source = [t.replace('~', name + '/') for t in source]
            target = [t.replace('~', name + '/') for t in target]
            for s, t in zip(source, target):
                self.loginfo('Remap {} -> {}'.format(s, t))
        if params:
            for n, v in params.items():
                self.loginfo('setparam {} -> {}'.format(n, v))
            rospy.set_param(name, params)
        try:
            self._load_srv.wait_for_service(timeout=self._srv_wait_timeout)
            res = self._load_srv(
                name=name,
                type=type,
                remap_source_args=source,
                remap_target_args=target,
                my_argv=my_argv or list(),
                bond_id=str()
            )
            if res.success:
                self.loginfo("Loaded {}".format(name))
                self._loaded_nodelets.add(name)
            else:
                self.logerr("Failed to load {}".format(name))
        except rospy.exceptions.ROSException:
            self.logerr("Failed to load {}".format(name))
            self.logerr(traceback.format_exc())
            return False
        except rospy.ServiceException:
            self.logerr("Failed to load {}".format(name))
            self.logerr(traceback.format_exc())
            return False

        return res.success

    def unload(self, name):
        if name not in self.loaded_nodelets:
            self.logwarn('nodelet {} is not loaded.'.format(name))
            return True
        try:
            self._unload_srv.wait_for_service(timeout=self._srv_wait_timeout)
            res = self._unload_srv(name=name)
            if res.success:
                self.loginfo("Unloaded {}".format(name))
                self._loaded_nodelets.remove(name)
            else:
                self.logerr("Failed to unload {}".format(name))
        except rospy.exceptions.ROSException:
            self.logerr("Failed to unload {}".format(name))
            self.logerr(traceback.format_exc())
            return False
        except rospy.ServiceException:
            self.logerr("Failed to load {}".format(name))
            self.logerr(traceback.format_exc())
            return False
        return res.success

    def unload_all(self):
        unloaded = False
        try:
            for name in self.loaded_nodelets:
                unloaded = self.unload(name) and unloaded
        except Exception:
            self.logerr("Failed to unload all nodelets")
            self.logerr(traceback.format_exc())
            unloaded = False
        return unloaded


class LoggerManager(NodeletManager):
    def __init__(self, name, color):
        super(LoggerManager, self).__init__(name=name, color=color)
        self.topics = set()

    def escape_topic(self, topic):
        name = topic
        if name.startswith('/'):
            name = name[1:]
        name = name.replace('/', '_') + '_logger'
        return name

    def watch(self, topic):
        if topic in self.topics:
            return False
        ok = self.load(
            name=self.escape_topic(topic),
            type="jsk_robot_lifelog/LightweightLogger",
            remappings={'~input': topic},
            params={'enable_monitor': False})
        if ok:
            self.loginfo('Started watching {}'.format(topic))
            self.topics.add(topic)
        else:
            self.logerr("Failed to watch {}".format(topic))
        return ok

    def unwatch(self, topic):
        if topic not in self.topics:
            return False
        ok = self.unload(name=self.escape_topic(topic))
        if ok:
            self.loginfo('Stopped watching {}'.format(topic))
            self.topics.remove(topic)
        else:
            self.logerr("Failed to unwatch {}".format(topic))
        return ok


class AutomatedLogger(object):
    def __init__(self, monitor_node, **kwargs):
        super(AutomatedLogger, self).__init__()
        self.managers = {}
        rospy.on_shutdown(self.shutdown_cb)

        default_manager_name = rospy.get_name() + "_manager"
        self.managers['default'] = LoggerManager.get_manager(default_manager_name)

        if not monitor_node.startswith('/'):
            raise ValueError('~monitor_node param must start with /')
        self.graph = Graph(monitor_node)

        host = kwargs.get('mongo_host', 'localhost')
        port = kwargs.get('mongo_port', 27017)
        database = kwargs.get('mongo_database')
        collection = kwargs.get('mongo_collection')
        client = pymongo.MongoClient(host, port)
        try:
            ok = (client[database].command('ping')['ok'] == 1.0)
            if not ok:
                raise RuntimeError()
        except:
            rospy.logfatal('Could not connect to database {}:{}/{}'.format(host, port, database))
            rospy.logfatal(traceback.format_exc())
            return
        self.log_db = client[database][collection]
        self.log_id = str(uuid.uuid1())

        self.standalone = kwargs.get('standalone', False)
        if self.standalone:
            rospy.loginfo('Standalone mode')

        self.record_log(rospy.Time.now(), 'start')

        update_rate = kwargs.get('update_rate', 1.0)
        self.update_timer = rospy.Timer(
            rospy.Duration(update_rate), self.update_cb)

    def record_log(self, stamp, event, **data):
        data.update({
            'stamp': {
                'secs': stamp.secs,
                'nsecs': stamp.nsecs,
                'seconds': float(stamp.secs) + float(stamp.nsecs) * 1e-9
            },
            'event': event,
            'monitor_node': self.graph.base_node,
            'logger_id': self.log_id,
        })
        try:
            self.log_db.insert(data)
            return True
        except Exception as e:
            rospy.logerr('Failed to insert record log: {}'.format(str(e)))
            rospy.logerr(traceback.format_exc())
            return False

    def shutdown_cb(self):
        for m in self.managers.values():
            m.stop_manager()
        try:
            self.record_log(rospy.Time.now(), 'shutdown')
        except Exception as e:
            rospy.logerr(e)

    def get_manager(self, topic):
        for m in self.managers.values():
            if topic in m.topics:
                return m

    def list_managers(self):
        suffix = '/load_nodelet'
        managers = [e.start for e in self.graph.node_srv_conn if e.end.endswith(suffix)]
        return managers

    def update_cb(self, event=None):
        stamp = event.current_real
        self.graph.update(upward=True, downward=False)
        topics = copy.deepcopy(self.graph.topics)
        node_topics = self.graph.node_topic_conn
        current = set()
        for m in self.managers.values():
            current = current | m.topics

        cancels = current - topics
        canceled = []
        if cancels:
            rospy.loginfo("Removing topics: {}".format(cancels))
        for topic in cancels:
            manager = self.get_manager(topic)
            if not manager:
                rospy.logerr('No manager found for topic {}'.format(topic))
                continue
            ok = manager.unwatch(topic)
            if ok:
                canceled.append(topic)
            else:
                rospy.logerr('failed to unwatch topic {} from manager {}'.format(
                    topic, manager.name))
        if canceled:
            self.record_log(stamp, 'unwatch', topics=canceled)

        adds = topics - current
        added = []
        managers = set(self.list_managers())
        if adds:
            rospy.loginfo("Adding topics: {}".format(adds))
        for topic in adds:
            pubs = [e.start for e in node_topics.find_edge_by_node(
                entopic(topic), upward=True, downward=False)]
            pub_set = set(pubs)
            pub_manager = managers & pub_set
            if pub_manager and not self.standalone:
                manager = list(pub_manager)[0]
            else:
                manager = 'default'
            if manager not in self.managers:
                self.managers[manager] = LoggerManager.get_manager(manager)
            ok = self.managers[manager].watch(topic)
            if ok:
                added.append(topic)
            else:
                rospy.logerr('failed to watch topic {} from manager {}'.format(
                    topic, self.managers[manager].name))
        if added:
            self.record_log(stamp, 'watch', topics=added)


def parse_args():
    if len(rospy.myargv()) == len(sys.argv):
        # launch from command line
        import argparse
        p = argparse.ArgumentParser()
        p.add_argument('monitor_node', help='Name of monitoring node')
        p.add_argument('--standalone', action='store_true')
        p.add_argument('--update-rate', '-r', help='Update rate')
        p.add_argument('--mongo-host', '--host', help='Mongodb hostname')
        p.add_argument('--mongo-port', '--port', help='Mongodb port')
        p.add_argument('--mongo-database', '--db', help='Mongodb database name')
        p.add_argument('--mongo-collection', '--col', help='Mongodb collection name to write log')
        args = p.parse_args()
        kwargs = {k: v for k, v in args._get_kwargs() if v is not None}
    else:
        # launch from roslaunch
        kwargs = {
            'monitor_node': rospy.get_param('~monitor_node'),
            'update_rate': rospy.get_param('~update_rate', None),
            'mongo_host': rospy.get_param('/mongodb_host', None),
            'mongo_port': rospy.get_param('/mongodb_port', None),
            'mongo_database': rospy.get_param('/robot/database'),
            'mongo_collection': rospy.get_param('~collection', None),
            'standalone': rospy.get_param('~standalone', None),
        }
        kwargs = {k: v for k, v in kwargs if v is not None}
    return kwargs

if __name__ == '__main__':
    rospy.init_node("automated_logger")
    kwargs = parse_args()
    rospy.loginfo('Arguments: %s' % kwargs)
    n = AutomatedLogger(**kwargs)
    rospy.spin()
