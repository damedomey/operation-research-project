from utils.graph import Graph


def get_cost_graph(graph: Graph):
    """
    Get the cost graph for the given graph.
    Insert new node if we have bidirectional arc with different cost
    """
    # todo: bidirectional arc with different cost

    cost_graph = Graph()
    for node in graph.get_nodes():
        for neighbor in graph.get_adjacent_nodes(node):
            cost_graph.add_node(node)
            cost_graph.add_node(neighbor)
            props = graph.get_edge_properties(node, neighbor)

            inverse = cost_graph.get_edge_properties(neighbor, node)
            if not inverse:
                cost_graph.add_edge(node, neighbor, props['capacity'], props['cost'])
            else:
                inter = "_" + node + "_" + neighbor + "_"
                cost_graph.add_node(inter)
                cost_graph.add_edge(node, inter, props['capacity'], props['cost'])
                cost_graph.add_edge(inter, neighbor, props['capacity'], 0)

    return cost_graph


def normalize(graph: Graph):
    pass
