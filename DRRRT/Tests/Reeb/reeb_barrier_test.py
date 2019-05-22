from ProblemConfigurations.Barriers import generate_barriers_problem, generate_barriers_reeb_graph
import matplotlib.pyplot as plt
from Drawing import draw_problem_configuration
from DRRRT import augment_reeb_graph

environment, robot, start, goal = generate_barriers_problem()
reeb_graph = generate_barriers_reeb_graph()

draw_problem_configuration(environment, robot, start, goal, draw_robot=False)
reeb_graph.draw("green")
plt.show()

reeb_graph, start_node, goal_node = augment_reeb_graph(environment, start, goal,
                                                       link_step_size=0.001 * environment.width,
                                                       reeb_graph=reeb_graph,
                                                       connection_distance_threshold=0.25 * environment.width)

# reeb_graph = prune_reeb_graph(reeb_graph, goal_node)

draw_problem_configuration(environment, robot, start, goal, draw_robot=False)
reeb_graph.draw('green')
plt.show()
