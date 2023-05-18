import sys
from utils.graph import *

if len(sys.argv) >= 2:
    absolute_filepath = sys.argv[1]
else:
    absolute_filepath = os.path.join(os.path.dirname(__file__), "data", "data.txt")

print("Computation will be done on " + absolute_filepath)

if not os.path.exists(absolute_filepath):
    exit("The file does not exist.")

graph = Graph.read_graph_from_file(absolute_filepath)

output = os.path.join(os.path.dirname(__file__), "out", "out")
print("Write the graph in file " + output)
graph.print_graph_image(output)

output = os.path.join(os.path.dirname(__file__), "out", "max flow graph")
gmax, flow = graph.edmonds_karp(graph.source, graph.sink)
print("Write the max flow graph in file " + output)
gmax.print_graph_image(output)
print("Flow max ", flow)

reachable, unreachable = graph.st_cut(graph.source, graph.sink)
print("s-t cut : ")
print("--> Reachable nodes : ", reachable)
print("--> Unreachable nodes : ", unreachable)
exit(0)
source = "A"
sink = "G"
g = Graph.read_graph_from_file(filepath)

g.print_graph_image("tmp2")
gmax, flow = g.edmonds_karp(source,sink)
gmax.print_graph_image("tmp2")
print("Flow max ", flow)
reachable, unreachable = g.st_cut(source, sink)
print("Reachable nodes : ", reachable)
print("Unreachable nodes : ", unreachable)