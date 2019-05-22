import numpy as np


class Environment:
    """ProblemConfigurations description of the environment, given by a list of polygons."""

    def __init__(self, polygons):
        self.polygons = polygons

        # Compute bounds of environment
        min_x, max_x, min_y, max_y = np.inf, -np.inf, np.inf, -np.inf
        for polygon in self.polygons:
            for vertex in polygon.vertices:
                min_x = min(min_x, vertex[0])
                max_x = max(max_x, vertex[0])
                min_y = min(min_y, vertex[1])
                max_y = max(max_y, vertex[1])
        self.coord_bounds = (min_x, max_x, min_y, max_y)
        self.width = max_x - min_x
        self.height = max_y - min_y
        self.num_clear_calls = 0

    def clear_coords(self, x, y):
        """Returns whether or not a point is in the environment's free space."""
        return all(not poly.contains_coords(x, y) for poly in self.polygons)

    def clear(self, robot):
        """Returns whether or not the robot is in the environment's free space."""
        self.num_clear_calls += 1
        return all(not robot.intersects_polygon(poly) for poly in self.polygons)

    def link_coords(self, position_start, position_end, step_size):
        """Returns whether or not the interpolated point sequence from start to end lies entirely in the
        environment's free space."""
        dist = np.linalg.norm(position_start - position_end)
        num_steps = max(np.ceil(dist / step_size), 3)
        for x, y in zip(np.linspace(position_start[0], position_end[0], num_steps),
                        np.linspace(position_start[1], position_end[1], num_steps)):
            if not self.clear_coords(x, y):
                return False
        return True

    def link(self, robot_start, robot_end, step_size):
        """Returns whether or not the interpolated configuration sequence from start to end lies entirely in the
        environment's free space."""
        workspace_pt1 = robot_start.configuration()
        workspace_pt2 = robot_end.configuration()

        dist = np.linalg.norm(workspace_pt1 - workspace_pt2)
        num_steps = max(np.ceil(dist / step_size), 3)
        # print("LINK: using {} steps".format(num_steps))

        robot = robot_start
        for x, y, angle in zip(np.linspace(workspace_pt1[0], workspace_pt2[0], num_steps),
                               np.linspace(workspace_pt1[1], workspace_pt2[1], num_steps),
                               np.linspace(workspace_pt1[2], workspace_pt2[2], num_steps)):
            robot.move(x, y, angle)
            if not self.clear(robot):
                return False

        return True

    def draw(self, color):
        for polygon in self.polygons:
            polygon.draw(color)

    def bounds(self):
        """Returns (min_x, max_x, min_y, max_y) bounds of environment."""
        return self.coord_bounds
