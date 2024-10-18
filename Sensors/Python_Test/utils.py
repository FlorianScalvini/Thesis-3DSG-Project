import math
import numpy as np

bound = [5.0108, 5.0803, 47.3002, 47.3373]
radiusEarth = 6371E3


class Node():
    def __init__(self, id, longitude, latitude):
        self.coor = GeographicPoint(longitude=longitude, latitude=latitude)
        self.id = id
        self._lst_way = []
        self.isIntersection = False

    def addWay(self, id_way):
        self._lst_way.append(id_way)
        if not self.isIntersection and len(self._lst_way) > 1:
            self.isIntersection = True

    def __repr__(self):
        return (f"Node {self.id} with a geographical coordonnate of ({self.coor.longitude}°,{self.coor.latitude}°)")

class Way():
    def __init__(self, id, attribut):
        self.id = id
        self.attribut = attribut
        self.lst_Node = []
        self.lst_Edge = []

    def addNode(self, node):
        self.lst_Node.append(node.id)
        node.addWay(self.id)

    def addEdge(self, edge):
        self.lst_Edge.append(edge)


class Edge():
    def __init__(self, node_A, node_B, weight):
        self.node_A = node_A
        self.node_B = node_B
        self.weight = weight

class GridCoordonnate():
    def __init__(self, bound, inc_long, inc_lat):
        self.node_id = {}
        self._minLong = bound[0]
        self._maxLong = bound[1]
        self._minLat = bound[2]
        self._maxLat= bound[3]
        self._inc_long = inc_long
        self._inc_lat = inc_lat
        self.nbGridLong = int(math.ceil((self._maxLong - self._minLong) / self._inc_long))
        self.nbGridLat = int(math.ceil((self._maxLat - self._minLat) / self._inc_lat))
        self.grid = np.empty((self.nbGridLong, self.nbGridLat), dtype=object)
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                self.grid[i, j] = []

    def addNode(self, node):
        if node.coor.inBound():
            x_grid = int(math.floor((node.coor.longitude - self._minLong) / self._inc_long))
            y_grid = int(math.floor((node.coor.latitude - self._minLat) / self._inc_lat))
            self.grid[x_grid, y_grid].append(node.id)

    def getNearestPointFromCoor(self, point, node_lst):
        if point.inBound():
            x_grid = int(math.floor((point.longitude - self._minLong) / self._inc_long))
            y_grid = int(math.floor((point.latitude - self._minLat) / self._inc_lat))
            node_nearest = node_lst[self.grid[x_grid][y_grid][0]]
            dist = GeographicPoint.toDistance(pointA=point, pointB=node_nearest.coor)
            for k in self.grid[x_grid][y_grid][1:]:
                current_node = node_lst[k]
                new_dist = GeographicPoint.toDistance(pointA=point, pointB=node_nearest.coor)
                if dist > new_dist:
                    dist = new_dist
                    node_nearest = current_node
            return node_nearest.id
        else:
            return None



class GeographicPoint:
    def __init__(self, longitude, latitude):
        self.longitude = longitude
        self.latitude = latitude

    def inBound(self):
        return bound[0] <= self.longitude <= bound[1] and  bound[2] <= self.latitude <= bound[3]

    @staticmethod
    def toDistance(pointA, pointB):
        dLat = math.radians(pointA.latitude - pointB.latitude)
        dLon = math.radians(pointA.longitude - pointB.longitude)
        phiA = math.radians(pointA.latitude)
        phiB = math.radians(pointB.latitude)
        a = math.sin(dLat / 2) ** 2 + math.cos(phiA) * math.cos(phiB) * math.sin(dLon / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        return radiusEarth * c

    @staticmethod
    def toNorthPosition(point, dist):
        new_latitude = point.latitude + (dist / radiusEarth) * (180 / math.pi)
        return GeographicPoint(longitude=point.longitude, latitude=new_latitude)

    @staticmethod
    def toEastPosition(point, dist):
        new_longitude = point.longitude + (dist / radiusEarth) * (180 / math.pi) / math.cos(point.latitude * math.pi / 180)
        return GeographicPoint(longitude=new_longitude, latitude=point.latitude)
