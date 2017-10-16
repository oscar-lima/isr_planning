#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
    packages=['knowledge_base_ros'], 
    package_dir={'knowledge_base_ros': 'ros/src/knowledge_base_ros'}
)

setup(**d)
