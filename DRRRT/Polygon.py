import shapely.geometry as geometry
from shapely import affinity
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as pltPolygon
from matplotlib.collections import PatchCollection
import numpy as np


def rectangle(left, right, top, bottom):
    return Polygon([(left, top), (right, top), (right, bottom), (left, bottom)])


class Polygon:
    """ProblemConfigurations polygon description that computes intersection and containment."""

    def __init__(self, vertices):
        self.vertices = vertices
        self.polygon = geometry.Polygon(vertices)

    def copy(self):
        return Polygon(self.vertices)

    def contains_point(self, pt):
        return self.polygon.contains(pt)

    def contains_coords(self, x, y):
        return self.contains_point(geometry.Point((x, y)))

    def intersects_polygon(self, other_poly):
        return self.polygon.intersects(other_poly.polygon)

    def intersects_vertex_list(self, poly_vertices):
        return self.intersects_polygon(Polygon(poly_vertices))

    def rotated(self, angle):
        return Polygon(affinity.rotate(self.polygon, angle * 180 / np.pi).exterior.coords)

    def __str__(self):
        return "Polygon: {}".format(self.vertices)

    def draw(self, color):
        ax = plt.gca()
        p = PatchCollection([pltPolygon(self.vertices)])
        p.set_facecolor(color)
        p.set_edgecolor("black")
        ax.add_collection(p)
