import matplotlib.pyplot as plt
import numpy as np
from Node import Node


class Tree:
    """Tree of configurations to be connected by closest distance."""

    def __init__(self, start_configuration=None, goal_configuration=None):
        if start_configuration is None or goal_configuration is None:
            self.root = None
            self.nodes = []
            self.goal = None
        else:
            self.root = Node(start_configuration)
            self.nodes = [self.root]
            self.goal = Node(goal_configuration, is_goal=True)

    def __len__(self):
        return len(self.nodes)

    def closest_node(self, other_node):
        closest_node, closest_dist = None, np.inf
        for node in self.nodes:
            current_dist = other_node.dist(node)
            if current_dist < closest_dist:
                closest_node, closest_dist = node, current_dist

        assert closest_node is not None
        return closest_node

    def add_node(self, node):
        self.nodes.append(node)

    def draw(self, color):
        xs = [node.configuration[0] for node in self.nodes]
        ys = [node.configuration[1] for node in self.nodes]
        plt.scatter(xs, ys, s=15, color=color)
        for node in self.nodes:
            if node.parent is not None:
                xs = [node.configuration[0], node.parent.configuration[0]]
                ys = [node.configuration[1], node.parent.configuration[1]]
                plt.plot(xs, ys, color=color)

    def final_path(self):
        """Returns tree containing only the final path to solve the motion planning query"""
        if self.nodes[-1].is_goal:
            # Backtrack to construct final path
            path_nodes = [self.goal]
            while path_nodes[-1].parent is not None:
                path_nodes.append(path_nodes[-1].parent)
            path_nodes = list(reversed(path_nodes))
            final_tree = Tree(self.root.configuration, self.goal.configuration)
            final_tree.nodes = path_nodes
            return final_tree
        else:
            return None
