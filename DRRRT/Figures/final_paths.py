from RRT import RRT
from DRRRT import DRRRT
from performance_tests import generate_problem, generate_reeb_graph
from Drawing import draw_final_path_frames
import matplotlib.pyplot as plt
from time import time
from multiprocessing import Pool, cpu_count


def generate_paths(name):
    environment, robot, start, goal = generate_problem(name)
    reeb_graph, start_node, goal_node = generate_reeb_graph(name)

    done = False
    start_time = time()
    while not done and name != 'maze':
        rrt_tree, _ = RRT(environment, robot, start, goal,
                          rrt_step_size=0.025 * environment.width,
                          link_step_size=0.005 * environment.width,
                          maximum_clear_calls=25000)
        done = rrt_tree.final_path() is not None

        if done:
            print("Ran RRT on {} in {:.2f} s".format(name, time() - start_time))
            plt.close()
            draw_final_path_frames(environment, robot, start, goal, rrt_tree, separate_frames=False, title='')
            plt.savefig("rrt_{}_path.pdf".format(name))

    done = False
    start_time = time()
    while not done:
        drrrt_tree, _ = DRRRT(environment, robot, start, goal,
                              rrt_step_size=0.025 * environment.width,
                              link_step_size=0.005 * environment.width,
                              reeb_graph=reeb_graph, start_node=start_node, goal_node=goal_node,
                              maximum_clear_calls=25000)
        done = drrrt_tree.final_path() is not None
        print("Ran DRRRT on {} in {:.2f} s".format(name, time() - start_time))

        if done:
            plt.close()
            draw_final_path_frames(environment, robot, start, goal, drrrt_tree, separate_frames=False, title='')
            plt.savefig("drrrt_{}_path.pdf".format(name))


pool = Pool(cpu_count())
pool.map(generate_paths, ['hallway', 'narrow', 'barriers', 'maze'])
