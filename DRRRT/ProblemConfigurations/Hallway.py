from Robot import Robot
from Polygon import Polygon, rectangle
from Environment import Environment
import numpy as np
from ReebGraph import construct_approximate_reeb_graph
from Graph import Graph


def generate_hallway_problem():
    """Generates a 'Hallway' problem configuration.

    Returns (environment, robot, start configuration, goal configuration)."""
    border = [rectangle(0, 500, 0, 10), rectangle(0, 500, 90, 100),
              rectangle(0, 10, 0, 100), rectangle(490, 500, 0, 100)]

    hallway_environment = Environment(border)
    robot_geometry = Polygon([(30 * np.sin(angle), -30 * np.cos(angle)) for angle in np.linspace(-np.pi, np.pi, 6)[1:]])
    robot = Robot(robot_geometry)
    start = np.array([45, 50, 0])
    goal = np.array([455, 50, np.pi])

    return hallway_environment, robot, start, goal


def generate_hallway_reeb_graph():
    environment, robot, start, goal = generate_hallway_problem()
    columns = construct_approximate_reeb_graph(environment, 100, 100)

    # Manual correction for better visualization
    columns.append([columns[0][0].copy()])
    columns[0][0].configuration[0] += 35
    columns[-1][0].configuration[0] += 435
    # End of manual correction

    reeb_graph = Graph([node for column in columns for node in column])

    # Connect nodes in graph
    reeb_graph.add_edge(columns[0][0], columns[-1][0])

    return reeb_graph
