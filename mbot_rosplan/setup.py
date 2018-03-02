#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['mbot_rosplan', 'mbot_rosplan_ros'],
 package_dir={'mbot_rosplan': 'common/src/mbot_rosplan', 'mbot_rosplan_ros': 'ros/src/mbot_rosplan_ros'}
)

setup(**d)
