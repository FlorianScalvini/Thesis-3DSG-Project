import lxml
import json
import xmltodict
import numpy as np
import math
import lxml.etree as etree
import matplotlib.pyplot as plt
import networkx as nx
from Graph import Map


map = Map("/home/ubuntu/Téléchargements/out.osm")
path = map.searchNearestPath("2861536902","251785105")
print("")
