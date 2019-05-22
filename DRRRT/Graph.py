import numpy as np
import matplotlib.pyplot as plt


def path_distances(graph, target):
    """Inefficient method for finding workspace distances to a target node in a graph."""
    distances = {n: np.inf for n in graph.nodes}
    distances[target] = 0
    done = False
    while not done:
        done = True
        for n, current_dist in list(distances.items()):
            for neighbor in graph.out_neighbors[n]:
                update_dist = 1 + distances[neighbor]
                if update_dist < current_dist:
                    distances[n] = update_dist
                    done = False
    return distances


class Graph:
    """Graph of configurations to be connected by closest distance."""

    def __init__(self, nodes):
        self.nodes = nodes
        self.in_neighbors = {n: [] for n in self.nodes}
        self.out_neighbors = {n: [] for n in self.nodes}

    def add_node(self, node):
        self.nodes.append(node)
        self.in_neighbors[node] = []
        self.out_neighbors[node] = []

    def add_edge(self, node1, node2):
        self.out_neighbors[node1].append(node2)
        self.in_neighbors[node2].append(node1)

    def remove_edge(self, node1, node2):
        self.out_neighbors[node1].remove(node2)
        self.in_neighbors[node2].remove(node1)

    def draw(self, color):
        xs = [node.configuration[0] for node in self.nodes]
        ys = [node.configuration[1] for node in self.nodes]
        plt.scatter(xs, ys, s=15, color=color)
        for node, neighbors in self.in_neighbors.items():
            for neighbor in neighbors:
                xs = [node.configuration[0], neighbor.configuration[0]]
                ys = [node.configuration[1], neighbor.configuration[1]]
                plt.plot(xs, ys, color=color)
