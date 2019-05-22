from Drawing import draw_problem_configuration
from Node import Node
import numpy as np


def construct_approximate_reeb_graph(environment, grid_x, grid_y):
    """Returns columns of points on a grid corresponding to the contracted level sets for a Reeb Graph.

    :param environment: Environment to construct Reeb Graph of
    :param grid_x: number of grid points in x direction
    :param grid_y: number of grid points in y direction
    :return: List[List[Node]]
    """
    min_x, max_x, min_y, max_y = environment.bounds()
    eps = 0.01 * environment.width
    min_x += eps
    min_y += eps
    max_x -= eps
    max_y -= eps

    draw_problem_configuration(environment, None, None, None, draw_robot=False)
    step_y = (max_y - min_y) / grid_y

    columns = []
    for x in np.linspace(min_x, max_x, grid_x):
        current_column = []

        y = min_y
        while y < max_y:
            start_y = y
            while environment.clear_coords(x, y) and y < max_y:
                y += step_y
            end_y = y - step_y

            if environment.clear_coords(x, start_y) and environment.clear_coords(x, end_y):
                current_column.append(Node(np.array([x, (start_y + end_y) / 2])))

            while not environment.clear_coords(x, y) and y < max_y:
                y += step_y

        columns.append(current_column)

    redundant_columns = set()
    for i in range(len(columns) - 1):
        if {n.configuration[1] for n in columns[i]} == {n.configuration[1] for n in columns[i + 1]}:
            redundant_columns.add(i + 1)
    columns = [columns[i] for i in range(len(columns)) if i not in redundant_columns and len(columns[i]) > 0]

    return columns
