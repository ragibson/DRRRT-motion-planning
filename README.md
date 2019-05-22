# DRRRT-motion-planning

(This README is temporary; more information will be added later)

This custom simulator is implemented in Python 3 and should run in any standard
Python interpreter.

We depend on the following libraries:
  - matplotlib: for figure generation and plotting
  - numpy: for general array/vector manipulation
  - shapely: for handling of planar polygons

All of which can be installed with pip (e.g. "pip install \<package name\>").

Most of the Python files contain helper classes. The important scripts are:
  - Tests/Reeb/reeb_barrier_test.py
    - Creates and plots the Reeb Graph example in Figure 1
  - Figures/sampling_region_visualization.py
    - Creates and plots the DRRRT growth example in Figure 2
  - Figures/problem_configurations.py
    - Creates and plots the problem configurations in Figure 3
  - performance_tests.py
    - Runs the experiments and saves the metrics shown in the table of Figure 4
  - Figures/performance_boxplots.py
    - Plots the boxplots of the experiment results in Figure 5
  - Figures/rrt_narrow_passages.py
    - Creates the "narrow passages RRT problem" figure from the presentation
  - Figures/rrt_hallway_suboptimal.py
    - Creates the "suboptimal RRT performance" figure from the presentation
  - Figures/final_paths.py
    - Creates the final paths figure at the end of the presentation

