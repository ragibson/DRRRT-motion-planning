from Robot import Robot
from Polygon import Polygon, rectangle
from Environment import Environment
import numpy as np
from ReebGraph import construct_approximate_reeb_graph
from Graph import Graph


def generate_split_problem():
    """Generates a 'Split' problem configuration.

    Returns (environment, robot, start configuration, goal configuration)."""
    walls = [rectangle(0, 400, 0, 10), rectangle(0, 400, 290, 300),
             rectangle(0, 10, 0, 300), rectangle(390, 400, 0, 300),
             rectangle(180, 220, 100, 200)]

    split_environment = Environment(walls)
    robot_geometry = Polygon([(-15, -15), (-15, 15), (15, 15), (15, -15)])
    robot = Robot(robot_geometry)
    start = np.array([50, 150, 0])
    goal = np.array([350, 150, 0])

    return split_environment, robot, start, goal


def generate_split_reeb_graph():
    environment, robot, start, goal = generate_split_problem()
    columns = construct_approximate_reeb_graph(environment, 100, 100)

    # Manual correction for better visualization
    columns = [columns[0], columns[1], [x.copy() for x in columns[1]], columns[2]]
    columns[0][0].configuration[0] = 125
    columns[3][0].configuration[0] = environment.width - 125
    for node in columns[1]:
        node.configuration[0] -= 5
    for node in columns[2]:
        node.configuration[0] += 40
    # End of manual correction

    reeb_graph = Graph([node for column in columns for node in column])

    # Connect nodes in graph
    for i in range(len(columns) - 1):
        if len(columns[i]) == 1:
            for node in columns[i + 1]:
                reeb_graph.add_edge(columns[i][0], node)
        elif len(columns[i + 1]) == 1:
            for node in columns[i]:
                reeb_graph.add_edge(node, columns[i + 1][0])
        else:
            for node1, node2 in zip(columns[i], columns[i + 1]):
                reeb_graph.add_edge(node1, node2)

    return reeb_graph
