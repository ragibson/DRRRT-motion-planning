from ProblemConfigurations.Barriers import generate_barriers_problem, generate_barriers_reeb_graph
from ProblemConfigurations.Hallway import generate_hallway_problem, generate_hallway_reeb_graph
from ProblemConfigurations.Narrow import generate_narrow_problem, generate_narrow_reeb_graph
from ProblemConfigurations.Split import generate_split_problem, generate_split_reeb_graph
from ProblemConfigurations.UniqueSolutionMaze import generate_maze_problem, generate_maze_reeb_graph
from RRT import RRT
from DRRRT import DRRRT, augment_reeb_graph
from multiprocessing import Pool, cpu_count
import numpy as np
import pickle
from Drawing import draw_full_tree
from time import time


def generate_problem(environment_name):
    if environment_name == 'barriers':
        environment, robot, start, goal = generate_barriers_problem()
    elif environment_name == 'hallway':
        environment, robot, start, goal = generate_hallway_problem()
    elif environment_name == 'narrow':
        environment, robot, start, goal = generate_narrow_problem()
    elif environment_name == 'split':
        environment, robot, start, goal = generate_split_problem()
    elif environment_name == 'maze':
        environment, robot, start, goal = generate_maze_problem()
    else:
        raise NotImplementedError("Environment {} not implemented.".format(environment_name))

    return environment, robot, start, goal


def generate_reeb_graph(environment_name):
    environment, robot, start, goal = generate_problem(environment_name)
    if environment_name == 'barriers':
        connection_distance_threshold = 0.25 * environment.width
        reeb_graph = generate_barriers_reeb_graph()
    elif environment_name == 'hallway':
        connection_distance_threshold = 0.25 * environment.width
        reeb_graph = generate_hallway_reeb_graph()
    elif environment_name == 'narrow':
        connection_distance_threshold = 0.3 * environment.width
        reeb_graph = generate_narrow_reeb_graph()
    elif environment_name == 'split':
        connection_distance_threshold = 0.25 * environment.width
        reeb_graph = generate_split_reeb_graph()
    elif environment_name == 'maze':
        connection_distance_threshold = 0.15 * environment.width
        reeb_graph = generate_maze_reeb_graph()
    else:
        raise NotImplementedError("Environment {} not implemented.".format(environment_name))

    reeb_graph, start_node, goal_node = augment_reeb_graph(environment, start, goal,
                                                           link_step_size=0.001 * environment.width,
                                                           reeb_graph=reeb_graph,
                                                           connection_distance_threshold=connection_distance_threshold)
    return reeb_graph, start_node, goal_node


def rrt_test(environment_name, maximum_clear_calls=25000):
    """Runs an RRT motion planner and returns (elapsed time, success, number of tree nodes, number of clear calls)."""
    np.random.seed()
    environment, robot, start, goal = generate_problem(environment_name)
    tree, elapsed_time = RRT(environment, robot, start, goal,
                             rrt_step_size=0.025 * environment.width,
                             link_step_size=0.005 * environment.width,
                             maximum_clear_calls=maximum_clear_calls)

    success = tree.final_path() is not None
    return elapsed_time, success, len(tree), environment.num_clear_calls


def drrrt_test(environment_name, maximum_clear_calls=25000):
    """Runs a DRRRT motion planner and returns (elapsed time, success, number of tree nodes, number of clear calls)."""
    np.random.seed()
    environment, robot, start, goal = generate_problem(environment_name)
    reeb_graph, start_node, goal_node = generate_reeb_graph(environment_name)
    tree, elapsed_time = DRRRT(environment, robot, start, goal,
                               rrt_step_size=0.025 * environment.width,
                               link_step_size=0.005 * environment.width,
                               reeb_graph=reeb_graph, start_node=start_node, goal_node=goal_node,
                               maximum_clear_calls=maximum_clear_calls)

    success = tree.final_path() is not None
    return elapsed_time, success, len(tree), environment.num_clear_calls


def print_summary(results):
    """Prints summary of list of RRT/DRRRT results (elapsed_time, success, num_nodes, num_clear_calls)."""
    print("Success rate: {:.2f}%".format(100 * sum(x[1] for x in results) / len(results)))
    print("Average time: {:.2f} s".format(np.average([x[0] for x in results])))
    print("Average nodes: {:.1f}".format(np.average([x[2] for x in results])))
    print("Average clear calls: {:.1f}".format(np.average([x[3] for x in results])))
    if any(x[1] for x in results):
        print("Average successful time: {:.2f} s".format(np.average([x[0] for x in results if x[1]])))
        print("Average successful nodes: {:.1f}".format(np.average([x[2] for x in results if x[1]])))
        print("Average successful clear calls: {:.1f}".format(np.average([x[3] for x in results if x[1]])))
    print()


def run_performance_tests(num_trials=8):
    pool = Pool(processes=cpu_count())
    for name in ['barriers', 'hallway', 'narrow', 'split', 'maze']:
        rrt_results = pool.map(rrt_test, [name] * num_trials)
        print("RRT results on {}:".format(name))
        print_summary(rrt_results)

        drrrt_results = pool.map(drrrt_test, [name] * num_trials)
        print("DRRRT results on {}:".format(name))
        print_summary(drrrt_results)

        pickle.dump(rrt_results,
                    open("/home/ryan/PycharmProjects/DRRRT/Output/rrt_{}_{}.p".format(name, num_trials), "wb"))
        pickle.dump(drrrt_results,
                    open("/home/ryan/PycharmProjects/DRRRT/Output/drrrt_{}_{}.p".format(name, num_trials), "wb"))

# start = time()
# run_performance_tests(1000)
# print("Full test suite took {:.2f} s".format(time() - start))
