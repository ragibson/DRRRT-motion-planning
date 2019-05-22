import numpy as np


class Node:
    """Node that contains a robot configuration and a parent node."""

    def __init__(self, configuration, parent=None, is_goal=False):
        self.configuration = configuration
        self.parent = parent
        self.is_goal = is_goal

    def copy(self):
        return Node(self.configuration.copy(), self.parent, self.is_goal)

    def dist(self, other_node):
        return np.linalg.norm(self.configuration - other_node.configuration)

    def __str__(self):
        if len(self.configuration) > 1:
            fmt = "Node: ({:.2f}" + ", {:.2f}" * (len(self.configuration) - 1) + ")"
            return fmt.format(*self.configuration)
        else:
            return "Node: {}".format(self.configuration)
