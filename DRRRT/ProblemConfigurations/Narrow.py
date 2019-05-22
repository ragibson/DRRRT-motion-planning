from Robot import Robot
from Polygon import Polygon, rectangle
from Environment import Environment
import numpy as np
from ReebGraph import construct_approximate_reeb_graph
from Graph import Graph


def generate_narrow_problem():
    """Generates a 'Narrow' problem configuration.

    Returns (environment, robot, start configuration, goal configuration)."""
    border = [rectangle(0, 400, 0, 10), rectangle(0, 400, 290, 300),
              rectangle(0, 10, 0, 300), rectangle(390, 400, 0, 300)]

    middle_obstacle = [rectangle(180, 220, 0, 200 - 17), rectangle(180, 220, 200 + 17, 300)]

    narrow_environment = Environment(border + middle_obstacle)
    robot_geometry = Polygon([(-15, -15), (-15, 30), (0, 30), (0, 0), (15, 0), (15, -15)])
    robot = Robot(robot_geometry)
    start = np.array([50, 50, 0])
    goal = np.array([350, 250, np.pi])

    return narrow_environment, robot, start, goal


def generate_narrow_reeb_graph():
    environment, robot, start, goal = generate_narrow_problem()
    columns = construct_approximate_reeb_graph(environment, 100, 100)

    # Manual correction for better visualization
    columns.append([columns[2][0].copy()])
    columns[2][0] = columns[1][0].copy()

    columns[0][0].configuration[0] += 40
    columns[1][0].configuration[0] -= 10
    columns[2][0].configuration[0] += 50
    columns[3][0].configuration[0] += 130
    columns.append([columns[2][0]])
    # End of manual correction

    reeb_graph = Graph([node for column in columns for node in column])

    # Connect nodes in graph
    for i in range(len(columns) - 1):
        reeb_graph.add_edge(columns[i][0], columns[i + 1][0])

    return reeb_graph
