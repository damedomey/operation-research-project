# Operation research project

This program compute : 
- Max flow and the list of flow value traversing each arc
- Min cut and the list of arc that form the minimum cut
- Max flow / min cost and the value of min cost

## Report

### Program Execution:
To run the program, the user can execute the "main.py" file. The program accepts 
an optional command-line argument, which specifies the path to the input file. 
If no argument is provided, a default file path is used. The input file should follow a specific format, as 
described in the project instructions. The program reads the graph from the input file, performs the necessary 
computations, and generates output files. 
- The output files are write in ./out/
- The default graph is in data/data.txt (white space is use as separator. It's the only one format that is accepted)

### Output and Visualization:
- Command line output 
- And Digraph pdf (in out/ directory)

### Important : 
Each node with capacity = 0 is ignored.

### Implementation Details:
The project was implemented using Python programming language. The program consists
of several modules, including "graph.py," "min_cost.py," and "main.py." 
The "graph.py" module defines the Graph class, which represents the directed graph 
and provides methods for adding nodes, edges, and retrieving graph information. 
The "min_cost.py" module contains the algorithms for minimum cost flow and related 
functions. The "main.py" file serves as the entry point of the program.

**Max flow:** The Edmonds-Karp algorithm is used to compute the maximum flow in the graph. 
It starts by initializing the residual graph, which is a representation of the remaining 
capacity on each edge in the original graph. Then, it repeatedly finds the shortest augmenting 
path in the residual graph using breadth-first search (BFS) and updates the flow along the path. 
The process continues until there are no more augmenting paths to be found. The maximum flow is
the sum of all the flows that were pushed along the augmenting paths.

**Min cost:** The minimum cost is computed using the Bellman-Ford algorithm. It starts by :
- Prepare the graph for the minimum cost computation. It creates a new graph called cost_graph 
and inserts a new node if there is a bidirectional arc between two nodes. This step ensures 
that the graph is ready for calculating the minimum cost flow. 
- The min_cost_flow(graph) function computes the minimum cost flow from the source to the sink 
in the given graph. It takes the prepared cost_graph as input.
Within the min_cost_flow function, the algorithm iteratively finds the minimum cost augmenting 
path using the min_cost_augmenting_path function of the graph object. It keeps sending flow along
the augmenting path until no more augmenting paths can be found.
During each iteration, the algorithm updates the flow in both the cost_graph and the original
input graph (graph). It keeps track of the maximum flow value (max_flow) achieved so far.
Once no more augmenting paths can be found, the algorithm terminates, and the cost_graph and 
max_flow are returned as the result. 
- The get_cost(graph) function computes the cost of the flow in the given graph. 

**Min cut:** The s-t cut represents a partition of the nodes in the graph into two sets: 
- the reachable nodes from the source (s) and 
- the unreachable nodes from the source. 

To find the s-t cut, the algorithm starts by computing the maximum flow in the graph. 
Then, it performs a breadth-first search from the source node to identify all reachable nodes. 
The unreachable nodes are simply the remaining nodes in the graph. Finally, it determines the set
of edges that form the minimum cut by iterating over all pairs of nodes, one from the reachable 
set and the other from the unreachable set, and checking if there is an edge between them in the
original graph.

**Cost:** The cost of the flow is computed by iterating over all nodes in the graph and 
their adjacent nodes. For each edge, it retrieves the edge properties, including the flow and 
cost. The cost of each edge is calculated by multiplying the flow on that edge with its associated
cost. The costs of all edges are summed up to obtain the total cost of the flow in the graph.
