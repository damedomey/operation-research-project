from utils.graph import Graph


def prepare(graph: Graph):
    """
    Prepare the graph for the min cost computation
    Insert new node if we have bidirectional arc between two nodes
    """
    cost_graph = Graph()
    cost_graph.source = graph.source
    cost_graph.sink = graph.sink
    for node in graph.get_nodes():
        for neighbor in graph.get_adjacent_nodes(node):
            cost_graph.add_node(node)
            cost_graph.add_node(neighbor)
            props = graph.get_edge_properties(node, neighbor)

            inverse = cost_graph.get_edge_properties(neighbor, node)
            if not inverse:
                cost_graph.add_edge(node, neighbor, props['capacity'], props['cost'], props['flow'])
            else:
                inter = "_" + node + "_" + neighbor + "_"
                cost_graph.add_node(inter)
                cost_graph.add_edge(node, inter, props['capacity'], props['cost'], props['flow'])
                cost_graph.add_edge(inter, neighbor, props['capacity'], 0, props['flow'])

    return cost_graph


def min_cost_flow(graph):
    """
        Compute the min cost flow from source to sink in the given graph
        Return the graph that have max flow, min cost
    """

    cost_graph = graph
    max_flow = 0
    internal_cost_graph = prepare(graph)
    while True:
        path, bottleneck = internal_cost_graph.min_cost_augmenting_path(internal_cost_graph.source, internal_cost_graph.sink)

        if path is None:
            break

        internal_cost_graph.send_flow_belong_path(path, bottleneck)
        cost_graph.send_flow_belong_path(path, bottleneck)
        max_flow += bottleneck
        internal_cost_graph = internal_cost_graph.residual()

    return cost_graph, max_flow
