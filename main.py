import os
import sys
from copy import deepcopy
from utils.graph import *
from utils.min_cost import min_cost_flow, get_cost, print_st_cut

if len(sys.argv) >= 2:
    absolute_filepath = sys.argv[1]
else:
    absolute_filepath = os.path.join(os.path.dirname(__file__), "data", "data.txt")

print("Computation will be done on " + absolute_filepath)

if not os.path.exists(absolute_filepath):
    exit("The file does not exist.")

initial_graph = Graph.read_graph_from_file(absolute_filepath)
graph = deepcopy(initial_graph)

output = os.path.join(os.path.dirname(__file__), "out", "out")
print("Write the graph in file " + output )
graph.print_graph_image(output)

output = os.path.join(os.path.dirname(__file__), "out", "max flow graph")
gmax, flow = graph.edmonds_karp(graph.source, graph.sink)
print("Write the max flow graph in file " + output)
gmax.print_graph_image(output)
print("Flow max ", flow)
print("The flow value traversing each arc is : ")
gmax.print_graph()

print_st_cut(graph)


# min cost
cost_graph, flow = min_cost_flow(initial_graph)
print("\nMin cost")
print("Write the min cost max flow graph in file ", output)
print("Max flow : ", flow)
output = os.path.join(os.path.dirname(__file__), "out", "min cost - max flow graph")
cost_graph.print_graph_image(output)
print("Min cost : ", get_cost(cost_graph))
print_st_cut(cost_graph)
print("The flow value traversing each arc is : ")
cost_graph.print_graph()
