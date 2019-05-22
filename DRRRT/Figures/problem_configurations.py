from performance_tests import generate_problem
from Drawing import draw_problem_configuration
import matplotlib.pyplot as plt

for name in ['barriers', 'hallway', 'narrow', 'split', 'maze']:
    environment, robot, start, goal = generate_problem(name)

    plt.close()
    draw_problem_configuration(environment, robot, start, goal, title='')
    plt.savefig("{}_problem.pdf".format(name))
