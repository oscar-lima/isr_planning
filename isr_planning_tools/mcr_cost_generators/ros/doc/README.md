Navigation cost generation
==========================

This package generates navigation cost information to be used by task planning.

It generates what is known as distance matrix, which is a summary of the distances between all locations.

To generate such data we loop trough all locations, calculating path length between them all (see Figure nav cost generation loop).

![alt text](navigation_cost_generation.png "navigation cost generation")

The end product is a file: costs.pddl which can be read directly by planners which support PDDL language.

to generate the costs launch the following components:

        roslaunch mir_bringup robot.launch
        roslaunch mcr_navigation_cost_generator navigation_cost_generator.launch

done.
