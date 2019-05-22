from performance_tests import generate_problem, generate_reeb_graph
from DRRRT import DRRRT

environment, robot, start, goal = generate_problem("split")
reeb_graph, start_node, goal_node = generate_reeb_graph("split")

tree, elapsed_time = DRRRT(environment, robot, start, goal,
                           rrt_step_size=0.05 * environment.width,
                           link_step_size=0.005 * environment.width,
                           reeb_graph=reeb_graph, start_node=start_node, goal_node=goal_node,
                           maximum_clear_calls=25000)

# In DRRRT, uncomment the sampling region visualization code block that starts with plt.close()
