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

    def __init__(self, raw_edges=[], raw_nodes={}):

        self.edges = {}
        self.nodes = {}
        self.network = nx.DiGraph()

        edges = []
        for raw_edge in raw_edges:
            edges.append( GraphEdge( raw_edge['from'],
                                     raw_edge['to'],
                                     raw_edge['behavior_type'],
                                     int(raw_edge['cost']),
                                     raw_edge['args'] ))
        nodes = {}
        for key, raw_node in raw_nodes.items():
            nodes[key] = GraphNode( key, raw_node )

        for key, node in nodes.items():
            self.nodes[key] = node
        for edge in edges:
            self.edges[edge.node_id_from,edge.node_id_to] = edge
            self.network.add_edge(
                                edge.node_id_from,
                                edge.node_id_to,
                                weight=edge.cost)

    def calcPath(self, node_id_from, node_id_to):

        try:
            node_id_list = nx.shortest_path( self.network, node_id_from, node_id_to )
        except nx.NetworkXNoPath as e:
            return None
        path = []
        for index in range(len(node_id_list)-1):
            path.append(self.edges[node_id_list[index],node_id_list[index+1]])
        return path
