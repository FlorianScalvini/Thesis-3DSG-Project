from utils import *
import networkx as nx
from lxml import etree
import matplotlib.pyplot as plt

bound = [5.0108, 5.0803, 47.3002, 47.3373]
x = 0.005
y = 0.0025


class Map():
    def __init__(self, path_osm):
        self.grid = GridCoordonnate(bound, x, y)
        tree = etree.parse(path_osm)
        root = tree.getroot()
        self.way_id = {}
        self.node_id = {}
        for child in root:
            if child.tag == "bounds":
                dict_bound = child.attrib
                bound_file = [float(dict_bound['minlon']), float(dict_bound['maxlon']), float(dict_bound['minlat']),
                              float(dict_bound['maxlat'])]
            if child.tag == "node":
                id = child.attrib["id"]
                self.node_id[id] = Node(id=id, longitude=float(child.attrib["lon"]), latitude=float(child.attrib["lat"]))
            if child.tag == "way":
                id = child.attrib["id"]
                self.way_id[id] = Way(id=id, attribut={})
                for child_2 in child:
                    if child_2.tag == "nd":
                        idx_node = child_2.attrib["ref"]
                        self.way_id[id].addNode(self.node_id[idx_node])
                        self.grid.addNode(self.node_id[idx_node])
                    else:
                        key = child_2.attrib["k"]
                        value = child_2.attrib["v"]
                        self.way_id[id].attribut[key] = value
        for k, value in self.way_id.items():
            e_0 = value.lst_Node[0]
            node_0 = self.node_id[e_0]
            for idx in value.lst_Node[1:-1]:
                if self.node_id[idx].isIntersection:
                    node_1 = self.node_id[idx]
                    dist = GeographicPoint.toDistance(node_0.coor, node_1.coor)
                    self.way_id[k].addEdge(Edge(node_A=e_0, node_B=idx, weight=dist))
                    node_0 = node_1
                    e_0 = idx
            node_1 = self.node_id[value.lst_Node[-1]]
            dist = GeographicPoint.toDistance(node_0.coor, node_1.coor)
            self.way_id[k].addEdge(Edge(node_A=e_0, node_B=value.lst_Node[-1], weight=dist))


        self.G = nx.Graph()
        for k, value in self.way_id.items():
            for edge in value.lst_Edge:
                node_1 = self.node_id[edge.node_A]
                node_2 = self.node_id[edge.node_B]
                if node_1.coor.inBound() and node_2.coor.inBound():
                    self.G.add_edge(edge.node_A, edge.node_B, weight=edge.weight)
        print("End init")


    def getWay(self):
        return self.way_id

    def searchNearestPath(self, node_A,  node_B):
        path = nx.dijkstra_path(self.G, node_A, node_B)
        return path

    def searchNearestPoint(self, longitude, latitude):
        return self.grid.getNearestPointFromCoor(GeographicPoint(longitude=longitude, latitude=latitude), self.node_id)

    def getNode(self, node):
        return self.node_id[node]

    def plotPathEdge(self, path):
        lst_edges = list(self.G.edges(data=True))
        for data in lst_edges:
            node_1 = self.node_id[data[0]]
            node_2 = self.node_id[data[1]]
            plt.plot([node_1.coor.longitude, node_2.coor.longitude],
                     [node_1.coor.latitude, node_2.coor.latitude], c='b')
        path_node = None
        for i in path:
            node = self.node_id[i]
            arr = np.array([node.coor.longitude, node.coor.latitude])
            if path_node is None:
                path_node = np.array([arr], dtype=float)
            else:
                path_node = np.append(path_node, [arr], axis=0)
        plt.plot(path_node[:, 0], path_node[:, 1], linewidth=5.0, c='r')
        plt.show()
