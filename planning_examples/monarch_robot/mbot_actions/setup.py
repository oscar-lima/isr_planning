#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mbot_actions_ros'],
    package_dir={'mbot_actions_ros': 'ros/src/mbot_actions_ros'}
)

setup(**d)
