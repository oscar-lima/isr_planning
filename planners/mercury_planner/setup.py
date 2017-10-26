#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mercury_planner'],
    package_dir={'mercury_planner': 'common/src/mercury_planner'}
)

setup(**d)
