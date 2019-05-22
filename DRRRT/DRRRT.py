import numpy as np
from time import time
from Tree import Tree
from Node import Node
from random import choice
from SamplingRegion import CircularSamplingRegion


class DRRRTRegion:
    """Region for DRRRT growth that includes a sampling region and source/target nodes in the Reeb Graph."""

    def __init__(self, source, target, radius):
        self.region = CircularSamplingRegion(source.configuration[0], source.configuration[1], radius)
        self.source = source
        self.target = target
        self.target_hit = False
        self.failures = 0

    def random_configuration(self):
        return self.region.random_configuration()

    def advance_region_distance(self, step_size):
        direction = self.target.configuration - self.source.configuration
        direction /= np.linalg.norm(direction)

        distance_to_target = np.linalg.norm(np.array([self.region.x, self.region.y]) - self.target.configuration)
        if step_size > distance_to_target:
            step_size = distance_to_target
            self.target_hit = True

        step = step_size * direction
        self.region.x += step[0]
        self.region.y += step[1]

    def advance_region_beyond(self, node, step_size):
        while self.region.contains_point(node.configuration[0], node.configuration[1]) and not self.target_hit:
            self.advance_region_distance(step_size)


def augment_reeb_graph(environment, start_configuration, goal_configuration, link_step_size,
                       reeb_graph, connection_distance_threshold):
    """Add start and goal configurations to a Reeb Graph."""
    start_node = Node(start_configuration[:2])
    goal_node = Node(goal_configuration[:2])

    start_neighbors = []
    goal_neighbors = []
    for n in reeb_graph.nodes:
        if np.linalg.norm(start_node.configuration - n.configuration) < connection_distance_threshold and \
                environment.link_coords(start_node.configuration, n.configuration, link_step_size):
            start_neighbors.append(n)

        if np.linalg.norm(goal_node.configuration - n.configuration) < connection_distance_threshold and \
                environment.link_coords(n.configuration, goal_node.configuration, link_step_size):
            goal_neighbors.append(n)

    reeb_graph.add_node(start_node)
    reeb_graph.add_node(goal_node)
    for start_neighbor in start_neighbors:
        reeb_graph.add_edge(start_node, start_neighbor)
    for goal_neighbor in goal_neighbors:
        reeb_graph.add_edge(goal_neighbor, goal_node)

    return reeb_graph, start_node, goal_node


def DRRRT(environment, robot, start_configuration, goal_configuration, rrt_step_size, link_step_size,
          reeb_graph, start_node, goal_node, region_failure_threshold=np.inf, maximum_clear_calls=np.inf):
    """
    Motion plan using a Dynamic Region-biased Rapidly-exploring Random Tree strategy.

    :param environment: Environment to motion plan in
    :param robot: Robot to plan motion for
    :param start_configuration: Starting robot configuration
    :param goal_configuration: Goal robot configuration
    :param rrt_step_size: Step size for RRT growth
    :param link_step_size: Step size for LINK computation
    :param reeb_graph: ReebGraph for RRT growth
    :param start_node: Start node of reeb graph
    :param goal_node: Goal node of reeb graph
    :param region_failure_threshold: Maximum number of failed queries before removing a region
    :param maximum_clear_calls: Timeout for query to fail
    :return: final tree, elapsed time
    """

    # Sampling regions and the graph edges they are traversing
    regions = []
    seen_edges = set()
    targets_hit = set()

    def add_regions_at_node(node):
        for neighbor in reeb_graph.out_neighbors[node]:
            if (node, neighbor) not in seen_edges and (neighbor, node) not in seen_edges:
                new_region = DRRRTRegion(node, neighbor, rrt_step_size)
                new_region.advance_region_distance(rrt_step_size)
                regions.append(new_region)
                seen_edges.add((node, neighbor))

        for i in reversed(range(len(regions))):
            if regions[i].target in targets_hit:
                del regions[i]

    done = False
    environment.num_clear_calls = 0
    tree = Tree(start_configuration, goal_configuration)
    add_regions_at_node(start_node)
    robot_next = robot.copy()
    start_time = time()

    while not done and environment.num_clear_calls < maximum_clear_calls:
        region = choice(regions)
        q_rand = region.random_configuration()
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

            region.advance_region_beyond(new_node, link_step_size)
            region.failures = 0

            if region.target_hit:
                if region.target is not goal_node:
                    regions.remove(region)
                    add_regions_at_node(region.target)
                    targets_hit.add(region.target)

            # plt.close()
            # for region in regions:
            #     region.region.draw('green')
            # reeb_graph.draw('black')
            # draw_full_tree(environment, robot, start_configuration, goal_configuration, tree, title='')
            # plt.savefig("tree{}.pdf".format(len(tree)))
        else:
            region.failures += 1
            if region.failures > region_failure_threshold:
                regions.remove(region)

    return tree, time() - start_time
