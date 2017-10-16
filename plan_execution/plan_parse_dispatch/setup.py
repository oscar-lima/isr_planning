#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['plan_parse_dispatch', 'plan_parse_dispatch_ros'],
 package_dir={'plan_parse_dispatch': 'common/src/plan_parse_dispatch', 'plan_parse_dispatch_ros': 'ros/src/plan_parse_dispatch_ros'}
)

setup(**d)
