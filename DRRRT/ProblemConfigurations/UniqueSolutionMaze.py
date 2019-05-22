from Robot import Robot
from Polygon import Polygon, rectangle
from Environment import Environment
import numpy as np
from Graph import Graph
from Node import Node


def generate_maze_problem():
    """Generates the 'UniqueSolutionMaze' problem configuration from OMPL.

    Returns (environment, robot, start configuration, goal configuration)."""
    environment_width = 800
    wall_width = 10
    grid = [int(round(x)) for x in np.linspace(0, environment_width, 11)]

    # horizontal walls: (y, x_start, x_end)
    horizontal_walls_grid = [(10, 0, 10), (9, 0, 1), (9, 3, 7), (9, 8, 10), (8, 1, 3), (8, 4, 5), (8, 6, 8), (8, 9, 10),
                             (7, 0, 1), (7, 2, 3), (7, 4, 5), (7, 6, 9), (6, 1, 2), (6, 3, 4), (6, 7, 9), (5, 2, 4),
                             (5, 5, 6), (5, 7, 9), (4, 0, 2), (4, 3, 5), (4, 9, 10), (3, 2, 6), (3, 7, 8), (3, 9, 10),
                             (2, 1, 2), (2, 6, 10), (1, 0, 1), (1, 5, 9), (0, 0, 10)]

    # vertical walls: (x, y_start, y_end)
    vertical_walls_grid = [(0, 0, 10), (1, 1, 2), (1, 3, 4), (1, 5, 6), (2, 0, 1), (2, 4, 5), (2, 6, 7), (2, 9, 10),
                           (3, 1, 3), (3, 6, 9), (4, 0, 2), (4, 5, 8), (5, 1, 3), (5, 4, 5), (5, 6, 7), (6, 3, 7),
                           (7, 2, 6), (7, 8, 9), (8, 3, 4), (8, 7, 8), (10, 0, 10)]

    # corner patches: (x, y)
    corners = [(5, 7), (7, 9), (8, 8), (10, 10)]

    walls = []
    for y, x_start, x_end in horizontal_walls_grid:
        walls.append(rectangle(grid[x_start], grid[x_end], grid[y], grid[y] + wall_width))
    for x, y_start, y_end in vertical_walls_grid:
        walls.append(rectangle(grid[x], grid[x] + wall_width, grid[y_start], grid[y_end]))
    for x, y in corners:
        walls.append(rectangle(grid[x], grid[x] + wall_width, grid[y], grid[y] + wall_width))

    maze_environment = Environment(walls)

    robot_geometry = Polygon([(-10, -30), (-10, 30), (10, 30), (10, -30)])
    robot = Robot(robot_geometry)
    start = np.array([60, 45, 0])
    goal = np.array([environment_width - 35, environment_width - 35, 0])

    return maze_environment, robot, start, goal


def generate_maze_reeb_graph():
    environment, robot, start, goal = generate_maze_problem()
    # columns = construct_approximate_reeb_graph(environment, 14, 100)

    # Manual definition of pruned reeb graph here
    environment_width = 800
    offset_grid = [int(round(x)) + 42.5 for x in np.linspace(0, environment_width, 11)]

    # nodes_grid: (x, y)
    nodes_grid = [(1, 0), (1, 1), (2, 1), (2, 0), (3, 0), (3, 2), (4, 2), (4, 0), (9, 0), (9, 1), (5, 1), (5, 2),
                  (6, 2), (6, 6), (9, 6), (9, 7), (8, 7), (8, 8), (7, 8), (7, 9), (8, 9)]
    nodes = [Node(np.array([offset_grid[x], offset_grid[y]])) for x, y in nodes_grid]
    reeb_graph = Graph(nodes)

    # Connect nodes in graph
    for i in range(len(nodes) - 1):
        reeb_graph.add_edge(nodes[i], nodes[i + 1])

    return reeb_graph
