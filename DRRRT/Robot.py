import numpy as np
from Polygon import Polygon


class Robot:
    """ProblemConfigurations robot description that stores the robot's geometry and current configuration."""

    def __init__(self, polygon, init_pos=np.array([0, 0]), init_rot=0):
        """
        :param polygon: Polygon of robot's polygon area
        :param init_pos: initial (x, y) position of robot
        :param init_rot: initial rotation angle of robot
        """
        self.geometry = polygon
        self.current_shape = None
        self.position = init_pos
        self.rotation = init_rot

    def copy(self):
        return Robot(self.geometry.copy(), self.position.copy(), self.rotation)

    def configuration(self):
        """Returns robot's (x, y, angle) configuration."""
        return np.array([self.position[0], self.position[1], self.rotation])

    def move(self, x=None, y=None, rot=None):
        if x is not None:
            self.position[0] = x

        if y is not None:
            self.position[1] = y

        if rot is not None:
            self.rotation = rot

        self.update()

    def update(self):
        self.current_shape = Polygon([(x + self.position[0], y + self.position[1])
                                      for (x, y) in self.geometry.vertices]).rotated(self.rotation)

    def intersects_polygon(self, other_poly):
        return self.current_shape.intersects_polygon(other_poly)

    def draw(self, color):
        self.current_shape.draw(color)
