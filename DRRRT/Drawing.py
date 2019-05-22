from Node import Node
import matplotlib.pyplot as plt
import numpy as np


def draw_problem_configuration(environment, robot, start, goal, draw_robot=True, title="Problem Configuration"):
    environment.draw('black')

    if draw_robot:
        robot_start, robot_goal = robot.copy(), robot.copy()
        robot_start.move(*start)
        robot_goal.move(*goal)
        robot_start.draw('red')
        robot_goal.draw('red')

    xs, xf, ys, yf = environment.bounds()
    plt.xlim([xs, xf])
    plt.ylim([ys, yf])
    plt.gca().set_aspect('equal')
    plt.axis("off")
    plt.tight_layout()
    plt.title(title)


def draw_full_tree(environment, robot, start, goal, tree, title="Final Tree"):
    draw_problem_configuration(environment, robot, start, goal, draw_robot=False)
    tree.draw('gray')

    final_path = tree.final_path()
    if final_path is not None:
        final_path.draw('green')

    plt.title(title)


def draw_final_path(environment, robot, start, goal, tree):
    final_path = tree.final_path()

    if final_path is not None:
        draw_problem_configuration(environment, robot, start, goal, draw_robot=False)

        for i in range(len(final_path) - 1):
            pt1 = final_path.nodes[i].configuration
            pt2 = final_path.nodes[i + 1].configuration

            # interpolate between configurations for display
            # Increase number of steps for large rotations (at least one step per 18 degrees)
            num_steps = max(5, np.ceil(abs(pt1[2] - pt2[2]) / (np.pi / 18)))

            for x, y, angle in zip(np.linspace(pt1[0], pt2[0], num_steps),
                                   np.linspace(pt1[1], pt2[1], num_steps),
                                   np.linspace(pt1[2], pt2[2], num_steps)):
                node = Node(np.array([x, y, angle]))
                robot.move(*node.configuration)
                robot.draw("red")


def draw_final_path_frames(environment, robot, start, goal, tree, title='Final Path', separate_frames=False):
    final_path = tree.final_path()
    draw_problem_configuration(environment, robot, start, goal, draw_robot=False, title=title)

    if final_path is not None:
        frame_count = 0

        for i in range(len(final_path) - 1):
            pt1 = final_path.nodes[i].configuration
            pt2 = final_path.nodes[i + 1].configuration

            # interpolate between configurations for display
            # Increase number of steps for large rotations (at least one step per 10 degrees)
            num_steps = max(5, np.ceil(abs(pt1[2] - pt2[2]) / (np.pi / 18)))

            for x, y, angle in zip(np.linspace(pt1[0], pt2[0], num_steps),
                                   np.linspace(pt1[1], pt2[1], num_steps),
                                   np.linspace(pt1[2], pt2[2], num_steps)):

                node = Node(np.array([x, y, angle]))
                robot.move(*node.configuration)
                frame_count += 1

                if separate_frames:
                    plt.close()
                    draw_problem_configuration(environment, robot, start, goal, draw_robot=False, title=title)
                    robot.draw("red")
                    plt.savefig("Output/final_path{}.png".format(frame_count), dpi=50)
                else:
                    robot.draw("red")
