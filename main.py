import os
import sys
from utils.graph import *
from utils.min_cost import *

if len(sys.argv) >= 2:
    absolute_filepath = sys.argv[1]
else:
    absolute_filepath = os.path.join(os.path.dirname(__file__), "data", "data.txt")

print("Computation will be done on " + absolute_filepath)

if not os.path.exists(absolute_filepath):
    exit("The file does not exist.")

output = os.path.join(os.path.dirname(__file__), "out", "out-tmp")
graph = Graph.read_graph_from_file(absolute_filepath)
cost_graph = min_cost_flow(graph)
cost_graph.print_graph_image(output)
exit(12)
reachable, unreachable, cut_edges = cost_graph.st_cut(cost_graph.source, cost_graph.sink)
print("\ns-t cut : ")
print("--> Reachable nodes : ", reachable)
print("--> Unreachable nodes : ", unreachable)
print("List of arc that form the minimum cut")
for source, target in cut_edges:
    print(source, " - ", target)
# print(cost_graph.min_cost_augmenting_path(cost_graph.source, cost_graph.sink))
exit(0)
output = os.path.join(os.path.dirname(__file__), "out", "out")
print("Write the graph in file " + output)
graph.print_graph_image(output)

output = os.path.join(os.path.dirname(__file__), "out", "max flow graph")
gmax, flow = graph.edmonds_karp(graph.source, graph.sink)
print("Write the max flow graph in file " + output)
gmax.print_graph_image(output)
print("Flow max ", flow)
print("The flow value traversing each arc is : ")
gmax.print_graph()

reachable, unreachable, cut_edges = graph.st_cut(graph.source, graph.sink)
print("\ns-t cut : ")
print("--> Reachable nodes : ", reachable)
print("--> Unreachable nodes : ", unreachable)
print("List of arc that form the minimum cut")
for source, target in cut_edges:
    print(source, " - ", target)
