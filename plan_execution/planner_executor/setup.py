#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['planner_executor', 'planner_executor_ros'],
 package_dir={'planner_executor': 'common/src/planner_executor', 'planner_executor_ros': 'ros/src/planner_executor_ros'}
)

setup(**d)
