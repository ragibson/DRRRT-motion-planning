import numpy as np
from time import time
from Tree import Tree
from Node import Node
from SamplingRegion import RectangularSamplingRegion


def RRT(environment, robot, start, goal, rrt_step_size, link_step_size, maximum_clear_calls=np.inf):
    """Shim to motion plan via RRT with sampling of entire workspace."""
    min_x, max_x, min_y, max_y = environment.bounds()
    sampling = RectangularSamplingRegion(min_x, max_x, min_y, max_y)
    return RRT_general(environment, robot, start, goal, rrt_step_size, link_step_size, sampling,
                       maximum_clear_calls=maximum_clear_calls)


def RRT_general(environment, robot, start, goal, rrt_step_size, link_step_size, sampling, maximum_clear_calls=np.inf):
    """
    Motion plan using a Rapidly-exploring Random Tree strategy.

    :param environment: Environment to motion plan in
    :param robot: Robot to plan motion for
    :param start: Starting robot configuration
    :param goal: Goal robot configuration
    :param rrt_step_size: Step size for RRT growth
    :param link_step_size: Step size for LINK computation
    :param sampling: SamplingRegion for RRT growth
    :param maximum_clear_calls: Timeout for query to fail
    :return: final tree, elapsed time
    """

    done = False
    environment.num_clear_calls = 0
    tree = Tree(start, goal)
    robot_next = robot.copy()
    start_time = time()

    while not done and environment.num_clear_calls < maximum_clear_calls:
        q_rand = sampling.random_configuration()
        q_near = tree.closest_node(q_rand)
        new_configuration = q_rand.configuration

        if q_near.dist(q_rand) > rrt_step_size:
            direction = q_rand.configuration - q_near.configuration
            direction /= np.linalg.norm(direction)
            new_configuration = q_near.configuration + rrt_step_size * direction

        robot.move(*q_near.configuration)
        robot_next.move(*new_configuration)
        if environment.link(robot, robot_next, link_step_size):
            new_node = Node(new_configuration, parent=q_near)
            tree.add_node(new_node)

            if new_node.dist(tree.goal) < rrt_step_size:  # TODO: and LINK?
                tree.goal.parent = new_node
                tree.add_node(tree.goal)
                done = True

    return tree, time() - start_time
