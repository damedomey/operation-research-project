import heapq
import re
from graphviz import Digraph

"""
    This is a class to manage the a directed graph.
"""


class Graph:
    def __init__(self):
        self.graph = {}
        self.source = None
        self.sink = None

    def add_node(self, node):
        if node not in self.graph:
            self.graph[node] = {}
            self.graph[node]['edge'] = {}
            self.graph[node]['description'] = {}

    def get_nodes(self):
        """ Return the list of nodes in the graph """
        return list(self.graph.keys())

    def add_edge(self, start, end, capacity, cost, flow=0):
        if capacity == 0:
            return
        if start in self.graph and end in self.graph:
            # FIXME: if this edge already exist ? replace or update or throw error ?
            self.graph[start]['edge'][end] = {'capacity': capacity, 'cost': cost, 'flow': flow}
        else:
            raise ValueError("Start or end node not in graph")

    def get_adjacent_nodes(self, node):
        if node in self.graph:
            return list(self.graph[node]['edge'].keys())
        else:
            return list()

    def get_edge_properties(self, start, end):
        if start in self.graph and end in self.graph[start]['edge']:
            return self.graph[start]['edge'][end]
        else:
            return None

    def update_edge_flow(self, start, end, flow):
        """ Increase the current flow with the new value if possible """
        if start in self.graph and end in self.graph[start]['edge']:
            current_flow = self.graph[start]['edge'][end]['flow']
            capacity = self.graph[start]['edge'][end]['capacity']
            if current_flow + flow <= capacity:
                self.graph[start]['edge'][end]['flow'] += flow
            else:
                raise RuntimeError(f"Unable to add the flow {flow} to the arc  {start} - {end} because the constraint "
                                   f"flow < capacity fail")
        else:
            raise ValueError("Start or end node not in graph")

    def print_graph(self):
        for node in self.graph:
            print(node, ":")
            for adjacent_node, edge_props in self.graph[node]['edge'].items():
                print("  ->", adjacent_node, ": capacity =", edge_props['capacity'], ", cost = ", edge_props['cost'],
                      ", flow =", edge_props['flow'])

    def print_graph_image(self, filename):
        dot = Digraph(comment='Graph write by David program')
        for node in self.graph:
            # Get node information
            label = self.graph[node]['description'].get('label', None)
            color = self.graph[node]['description'].get('color', None)
            dot.node(str(node), label=label, color=color)
            for adjacent_node, edge_props in self.graph[node]['edge'].items():
                color = self.graph[node]['edge'][adjacent_node].get('color', 'black')
                edge_label = f"<<font color='{color}'>{edge_props['flow']}/{edge_props['capacity']} ({edge_props['cost']})</font>>"
                dot.edge(str(node), str(adjacent_node), label=edge_label, color=color)
            dot.render(filename, view=False, format='pdf')

    def read_graph_from_file(filepath: str):
        """
            Reads a graph from a file (in the format describe in project

            :param filepath : Path to file. It's recommend to give a absolute path
        """
        with open(filepath, "r") as f:
            content = f.read()

        graph = Graph()
        for idx, line in enumerate(content.splitlines()):
            if idx == 0:
                nb_node, nb_arc, source, sink = line.split()
                graph.source = source
                graph.sink = sink
                continue

            start, end, capacity, cost = line.split()
            graph.add_node(start)
            graph.add_node(end)
            graph.add_edge(start, end, int(capacity), int(cost))

        return graph

    def residual(graph):
        residual_graph = Graph()
        residual_graph.source = graph.source
        residual_graph.sink = graph.sink
        for node in graph.graph.keys():
            residual_graph.add_node(node)
        for node in graph.graph:
            for adjacent_node, edge_props in graph.graph[node]['edge'].items():
                capacity = int(edge_props['capacity'])
                flow = int(edge_props['flow'])
                cost = int(edge_props['cost'])
                if capacity > 0:
                    residual_capacity = capacity - flow
                    residual_graph.add_edge(node, adjacent_node, residual_capacity, cost)
                    residual_graph.add_edge(adjacent_node, node, flow, -cost)
        return residual_graph

    def edmonds_karp(self, source, sink):
        """ 
        Compute the max flow. 
        Return the maximum graph and max flow value
        """
        residual = self.residual()
        max_flow = 0

        while True:
            # find the shortest augmenting path in the residual graph using BFS
            path = self.shortest_augmenting_path(residual, source, sink)
            if path is None:
                break

            # compute the bottleneck capacity of the path
            bottleneck = float("inf")
            for i in range(len(path) - 1):
                edge_props = residual.get_edge_properties(path[i], path[i + 1])
                if edge_props['capacity'] < bottleneck:
                    bottleneck = edge_props['capacity']
            for i in range(len(path) - 1):
                try:
                    self.update_edge_flow(path[i], path[i + 1], bottleneck)
                except:
                    self.update_edge_flow(path[i + 1], path[i], -bottleneck)

            residual = self.residual()

            # update the maximum flow
            max_flow += bottleneck

        return self, max_flow

    def shortest_augmenting_path(self, residual, source, sink):
        queue = [source]
        visited = set()
        parent = {}

        while len(queue) > 0:
            node = queue.pop(0)
            visited.add(node)

            # find the neighbors of the node in the residual graph
            for neighbor in residual.get_adjacent_nodes(node):
                # if the neighbor has not been visited and the capacity is positive
                if neighbor not in visited and residual.get_edge_properties(node, neighbor)['capacity'] > 0:
                    queue.append(neighbor)
                    parent[neighbor] = node

                    if neighbor == sink:
                        path = [sink]
                        while path[-1] != source:
                            path.append(parent[path[-1]])
                        path.reverse()
                        return path

        return None

    def st_cut(self, source, sink):
        """ 
        Find the s/t cut in the graph. The current graph must be a maximum graph.
        Return the list of reachable, unreachable and arc that form min cut
        """
        residual = self.residual()
        queue = [source]
        visited = set()

        while len(queue) > 0:
            node = queue.pop(0)
            visited.add(node)

            for neighbor in residual.get_adjacent_nodes(node):
                if neighbor not in visited and residual.get_edge_properties(node, neighbor)['capacity'] > 0:
                    queue.append(neighbor)

                    if neighbor == sink:
                        raise RuntimeError("The sink is reachable so this graph isn't")
        reachable = list(visited)
        unreachable = list(set(self.get_nodes()) - visited)
        reachable.sort()
        unreachable.sort()

        # find the list of arc that form the min cut
        cut_edges = []
        for source in reachable:
            for target in unreachable:
                x = self.get_edge_properties(source, target)
                if x:
                    cut_edges.append((source, target))

        return reachable, unreachable, cut_edges

    def bellman_ford(self, start):
        distances = {node: float('inf') for node in self.get_nodes()}
        distances[start] = 0
        parents = {node: None for node in self.get_nodes()}

        for _ in range(len(self.get_nodes()) - 1):
            for node in self.get_nodes():
                for end in self.get_adjacent_nodes(node):
                    edge = self.get_edge_properties(node, end)
                    distance = distances[node] + edge['cost']
                    if distance < distances[end]:
                        distances[end] = distance
                        parents[end] = node

        # Check for negative cycles
        for node in self.get_nodes():
            for end in self.get_adjacent_nodes(node):
                edge = self.get_edge_properties(node, end)
                distance = distances[node] + edge['cost']
                if distance < distances[end]:
                    raise ValueError("Graph contains a negative cycle")
        return distances, parents

    def min_cost_augmenting_path(self, source, sink):
        """
        Compute the minimum cost or capacity augmenting path using Dijkstra's algorithm.
        :param source
        :param sink
        Return the path as a list of nodes and the cost of the path.
        """
        distances, previous = self.bellman_ford(source)

        if distances[sink] == float('inf'):
            return None, None

        path = []
        capacities = []
        bottleneck = 0
        current_node = sink
        prev = None
        while current_node is not None:
            if prev is not None:
                props = self.get_edge_properties(current_node, prev)
                capacities.append(props['capacity'] - props['flow'])
            path.append(current_node)
            prev = current_node
            current_node = previous[current_node]

        path.reverse()
        bottleneck = min(capacities)

        return path, bottleneck

    def send_flow_belong_path(self, augmenting_path: list, bottleneck: int):
        for i in range(len(augmenting_path) - 1):
            try:
                self.update_edge_flow(augmenting_path[i], augmenting_path[i + 1], bottleneck)
            except:
                pass


if __name__ == '__main__':
    g = Graph()

    # add some nodes
    g.add_node("s")
    g.add_node("u")
    g.add_node("v")
    g.add_node("t")

    # add some edges with capacity and flow
    g.add_edge("s", "u", 10, 0)
    g.add_edge("s", "v", 5, 0)
    g.add_edge("u", "v", 15, 0)
    g.add_edge("u", "t", 5, 0)
    g.add_edge("v", "t", 10, 0)

    g.print_graph()
    g_max, max_flow = g.edmonds_karp("s", "t")
    print("Max flow ", max_flow)
