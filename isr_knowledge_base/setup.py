#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
    packages=['isr_knowledge_base_ros'], 
    package_dir={'isr_knowledge_base_ros': 'ros/src/isr_knowledge_base_ros'}
)

setup(**d)
