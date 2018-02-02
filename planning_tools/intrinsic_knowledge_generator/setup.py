#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['intrinsic_knowledge_generator', 'intrinsic_knowledge_generator_ros'],
 package_dir={'intrinsic_knowledge_generator': 'common/src/intrinsic_knowledge_generator', 'intrinsic_knowledge_generator_ros': 'ros/src/intrinsic_knowledge_generator_ros'}
)

setup(**d)
