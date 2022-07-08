# -*- coding: utf-8 -*-

import networkx as nx


class GraphEdge:

    def __init__(self,
                 node_id_from,
                 node_id_to,
                 behavior_type,
                 cost,
                 properties
                 ):
        self.node_id_from = node_id_from
        self.node_id_to = node_id_to
        self.behavior_type = behavior_type
        self.cost = cost
        self.properties = properties


class GraphNode:

    def __init__(self,
                 node_id,
                 properties
                 ):
        self.node_id = node_id
        self.properties = properties


class SupportBehaviorGraph:

    # 現在の SupportBehaviorGraph の仕様
    #   重み付きの有向グラフ
    #   あるノードからあるノードまでのエッジの数は 0 or 1

    def __init__(self, raw_edges=[], raw_nodes={}):

        self.edges = {}
        self.nodes = {}
        self.network = nx.DiGraph()

        for raw_edge in raw_edges:
            self.add_edge(raw_edge['from'],
                          raw_edge['to'],
                          raw_edge['behavior_type'],
                          int(raw_edge['cost']),
                          raw_edge['args'])

        for key, raw_node in raw_nodes.items():
            self.add_node(key, raw_node)

    def calcPath(self, node_id_from, node_id_to):

        try:
            node_id_list = nx.shortest_path(
                self.network, node_id_from, node_id_to)
        except nx.NetworkXNoPath as e:
            return None
        path = []
        for index in range(len(node_id_list)-1):
            path.append(self.edges[node_id_list[index], node_id_list[index+1]])
        return path

    def add_node(self,
                 node_id,
                 properties):
        # TODO: もしすでにnodeがあれば,適切に上書きするようにする
        # NOTICE: node と edge で整合性を保つようには実装されていない
        self.nodes[node_id] = GraphNode(node_id, properties)

    def add_edge(self,
                 node_id_from,
                 node_id_to,
                 behavior_type,
                 cost,
                 args):
        # TODO: もしすでにedgeがあれば,適切に上書きするようにする
        # NOTICE: node と edge で整合性を保つようには実装されていない
        edge = GraphEdge(node_id_from,
                         node_id_to,
                         behavior_type,
                         cost,
                         args)
        self.edges[node_id_from, node_id_to] = edge
        self.network.add_edge(node_id_from,
                              node_id_to,
                              weight=cost)

    def remove_node(self,
                    node_id):
        del self.nodes[node_id]

    def remove_edge(self,
                    node_id_from,
                    node_id_to):
        del self.edges[node_id_from, node_id_to]
        self.network.remove_edge(node_id_from, node_id_to)
