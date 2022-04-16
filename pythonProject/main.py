from graph_manager import GraphManager
import networkx as nx
import json
import math
import random
import time
from collections import deque
import matplotlib as plt
with open("./dati-json/dpc-covid19-ita-province.json") as f:
    parsed_file = json.load(f)

# Building the graph of provinces
provinces_already_annotated = []
P = GraphManager()
for province_data in parsed_file:
    # extracting information from the JSON
    if province_data['sigla_provincia'] != '' and province_data[
        'sigla_provincia'] not in provinces_already_annotated:
        provinces_already_annotated.append(province_data['sigla_provincia'])
        province = province_data['denominazione_provincia']
        position_x = province_data['long']
        position_y = province_data['lat']
        # adding each province to the graph
        P.add_node_to_graph(province, position_x, position_y)

# Building the graph of doubles
R = GraphManager()
# Generate 2000 pairs of double (x,y)
for i in range(2000):
    x = round(random.uniform(30, 50), 1)
    y = round(random.uniform(10, 20), 1)
    R.add_node_to_graph(str(i), x, y)
# Inserting the edges according to the distance between each node
%timeit P.add_edges()
%timeit R.add_edges()
def bellman_ford_SPFA(self, source_vertex):
    distances = dict.fromkeys(self.graph.nodes(), math.inf)
    already_in_queue = dict.fromkeys(self.graph.nodes(), False)
    predecessors = dict.fromkeys(self.graph.nodes(), math.inf)
    distances[source_vertex] = 0

    q = deque()
    q.append(source_vertex)
    already_in_queue[source_vertex] = True
    while len(q) > 0:
        u = q.popleft()
        for (u, v) in self.graph.edges(u):
            if distances[u] + float(self.graph[u][v]['label']) < distances[v]:
                distances[v] = distances[u] + float(self.graph[u][v]['label'])
                if not already_in_queue[v]:
                    q.append(v)
                    already_in_queue[v] = True
                predecessors[v] = u
            if distances[v] + float(self.graph[v][u]['label']) < distances[u]:
                distances[u] = distances[v] + float(self.graph[v][u]['label'])
                if not already_in_queue[u]:
                    q.append(u)
                    already_in_queue[u] = True
                predecessors[u] = v
    return distances, predecessors
%timeit P.bellman_ford("Firenze")
%timeit P.bellman_ford_SPFA("Firenze")
distances, predecessors = P.bellman_ford_SPFA("Firenze")
distances
predecessors
def bellman_ford_shortest_path(self, source_vertex, SPFA=True):
    if SPFA:
        distances, predecessors = self.bellman_ford_SPFA(source_vertex)
    else:
        distances, predecessors = self.bellman_ford(source_vertex)
    all__shortest_paths = []
    nodes = list(self.graph.nodes(data=True))
    for target_vertex in nodes:
        target_vertex = target_vertex[0]
        if predecessors[target_vertex] == math.inf:
            continue
        # using the predecessors of each node to build the shortest path
        shortest_path = []
        current_node = target_vertex
        shortest_path.append(target_vertex)
        while current_node != source_vertex:
            current_node = predecessors[current_node]
            # no path between the two nodes: exiting from the loop
            if current_node is None:
                break
            shortest_path.append(current_node)
        if len(shortest_path) != 1:
            all__shortest_paths.append(shortest_path)
    return all__shortest_paths
P.bellman_ford_shortest_path("Firenze", SPFA=True)[:5]
%timeit P.bellman_ford_shortest_path("Firenze", SPFA=True)
# Building the TOY graph of provinces (for example, slice [75:85])
provinces_already_annotated = []
P_toy = GraphManager()
for province_data in parsed_file[75:85]:
    # extracting information from the JSON
    if province_data['sigla_provincia'] != '' and province_data[
        'sigla_provincia'] not in provinces_already_annotated:
        provinces_already_annotated.append(province_data['sigla_provincia'])
        province = province_data['denominazione_provincia']
        position_x = province_data['long']
        position_y = province_data['lat']
        # adding each province to the graph
        P_toy.add_node_to_graph(province, position_x, position_y)
P_toy.add_edges()