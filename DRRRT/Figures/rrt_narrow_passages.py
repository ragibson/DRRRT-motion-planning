from RRT import RRT
from Drawing import draw_full_tree
import matplotlib.pyplot as plt
from performance_tests import generate_problem, generate_reeb_graph

environment, robot, start, goal = generate_problem("narrow")
reeb_graph = generate_reeb_graph("narrow")

done = False
while not done:
    print("Running RRT instance...")
    tree, elapsed_time = RRT(environment, robot, start, goal,
                             rrt_step_size=0.025 * environment.width,
                             link_step_size=0.005 * environment.width,
                             maximum_clear_calls=25000 // 2)
    done = tree.final_path() is None

plt.close()
draw_full_tree(environment, robot, start, goal, tree, title="")
plt.savefig("rrt_narrow_passage_failure.pdf")
