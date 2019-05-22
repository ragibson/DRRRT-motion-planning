from RRT import RRT
from Drawing import draw_full_tree
from Tree import Tree
import matplotlib.pyplot as plt
from performance_tests import generate_problem, generate_reeb_graph

environment, robot, start, goal = generate_problem("hallway")
reeb_graph = generate_reeb_graph("hallway")
largest_tree = Tree()
for i in range(500):
    tree, elapsed_time = RRT(environment, robot, start, goal,
                             rrt_step_size=0.025 * environment.width,
                             link_step_size=0.005 * environment.width,
                             maximum_clear_calls=25000 // 2)
    if len(tree) > len(largest_tree):
        largest_tree = tree
        print("Iteration {} found a larger tree with {} nodes".format(i, len(largest_tree)))

plt.close()
draw_full_tree(environment, robot, start, goal, largest_tree, title="")
plt.savefig("rrt_hallway_tree.pdf")
