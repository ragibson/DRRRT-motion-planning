from Robot import Robot
from Polygon import Polygon, rectangle
from Environment import Environment
import numpy as np
from ReebGraph import construct_approximate_reeb_graph
from Graph import Graph


def generate_barriers_problem():
    """Generates the 'Barriers' problem configuration from OMPL.

    Returns (environment, robot, start configuration, goal configuration)."""
    border = [rectangle(28, 1298, 52, 18), rectangle(28, 62, 865, 18),
              rectangle(1264, 1298, 865, 18), rectangle(28, 1298, 865, 832)]
    column1 = [rectangle(186, 226, 765, 650), rectangle(186, 226, 560, 447), rectangle(186, 226, 366, 252)]
    column2 = [rectangle(344, 382, 624, 510), rectangle(344, 382, 429, 316), rectangle(344, 382, 234, 122)]
    column3 = [rectangle(500, 540, 765, 650), rectangle(500, 540, 560, 447), rectangle(500, 540, 170, 52)]
    column4 = [rectangle(682, 720, 624, 510), rectangle(682, 720, 429, 316), rectangle(682, 720, 234, 122)]
    column5 = [rectangle(874, 914, 765, 650), rectangle(874, 914, 560, 447), rectangle(874, 914, 366, 252)]
    column6 = [rectangle(1058, 1096, 624, 510), rectangle(1058, 1096, 429, 316), rectangle(1058, 1096, 234, 122)]

    barriers_environment = Environment(border + column1 + column2 + column3 + column4 + column5 + column6)
    robot_geometry = Polygon([(-33, 63), (-33, -63), (33, -63), (33, -42), (-11, -42),
                              (-11, 42), (33, 42), (33, 63), (-33, 63)])
    robot = Robot(robot_geometry)
    start = np.array([115, 735, 0])
    goal = np.array([1180, 146, np.pi])

    return barriers_environment, robot, start, goal


def generate_barriers_reeb_graph():
    environment, robot, start, goal = generate_barriers_problem()
    columns = construct_approximate_reeb_graph(environment, 100, 100)

    # Manual correction for better visualization
    new_columns = []
    for c in columns:
        if len(c) == 1:
            new_columns.append(c)
        else:
            c_copy = [n.copy() for n in c]
            for n in c_copy:
                n.configuration[0] -= 20
            for n in c:
                n.configuration[0] += 40
            new_columns.append(c_copy)
            new_columns.append(c)
    columns = new_columns

    for i in range(0, len(columns), 3):
        for n in columns[i]:
            n.configuration[0] += 50
    columns[-1][0].configuration[0] += 50
    # End of manual correction

    reeb_graph = Graph([node for column in columns for node in column])

    # Connect nodes in graph
    for i in range(1, len(columns)):
        for parent in columns[i - 1]:
            if len(columns[i - 1]) == 1 or len(columns[i]) == 1:
                # Fully connect chokepoints
                for child in columns[i]:
                    reeb_graph.add_edge(parent, child)
            else:
                # Connect redundant columns directly
                assert len(columns[i - 1]) == len(columns[i])
                for j in range(len(columns[i])):
                    reeb_graph.add_edge(columns[i - 1][j], columns[i][j])

    return reeb_graph
