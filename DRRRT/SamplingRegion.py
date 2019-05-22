import numpy as np
from Node import Node
import matplotlib.pyplot as plt


class SamplingRegion:
    def random_configuration(self):
        raise NotImplementedError("random_configuration not implemented.")


class RectangularSamplingRegion(SamplingRegion):
    """Random sampling of configurations from a rectangular region."""

    def __init__(self, min_x=0, max_x=0, min_y=0, max_y=0):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

    def random_configuration(self):
        x = np.random.uniform(self.min_x, self.max_x)
        y = np.random.uniform(self.min_y, self.max_y)
        angle = np.random.uniform(-np.pi, np.pi)
        return Node(np.array([x, y, angle]))


class CircularSamplingRegion(SamplingRegion):
    """Random sampling of configurations from a circular region."""

    def __init__(self, init_x=0, init_y=0, init_radius=1):
        self.x = init_x
        self.y = init_y
        self.radius = init_radius
        self.radius_sqr = self.radius ** 2

    def random_configuration(self):
        length = np.sqrt(np.random.uniform(0, self.radius_sqr))
        workspace_angle = np.random.uniform(0, 2 * np.pi)

        x = self.x + length * np.cos(workspace_angle)
        y = self.y + length * np.sin(workspace_angle)
        angle = np.random.uniform(-np.pi, np.pi)
        return Node(np.array([x, y, angle]))

    def contains_point(self, x, y):
        return np.sqrt((self.x - x) ** 2 + (self.y - y) ** 2) < self.radius

    def draw(self, color):
        circ = plt.Circle((self.x, self.y), self.radius, color=color, alpha=0.5)
        plt.gca().add_artist(circ)
        circ_border = plt.Circle((self.x, self.y), self.radius, color=color, fill=False, linewidth=2)
        plt.gca().add_artist(circ_border)
