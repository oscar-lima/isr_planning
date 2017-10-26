mercury_planner
===============

This documentation is deprecated and needs serious attention !
Use it only as template to generate a complete new one from scratch

Mercury planner is a task planner from International Planning Competition 2014 (IPC) created by

Michael Katz and Jörg Hoffmann from Saarland University, Germany. The source of the planner is

provided under the following url:

        https://helios.hud.ac.uk/scommv/IPC-14/repo_planners/Mercury-fixed.zip

This package downloads that file, extracts only the sequential satisfactory folder (seq-sat-mercury)

and compiles it.

The dependencies of the planner are:

          sudo apt-get install bison flex gawk g++-multilib pypy

NOTE: Keep in mind that the script repository.debs at the root of mas_third_party_software installs

those dependencies along with all the mas_third_party_software dependencies for the rest of the packages.


Manual installation of the planner
==================================

If you want to perform a manual installation of the planner, you could follow the instructions under

        manual_installation.md

About this node
===============

Uses mcr_run_script_tool to call an external script located under :

When you publish event_in (see A.Test the component) the script gets called and mercury planner

pipeline is triggered (translate -> preprocess -> search).

As a result of this and if the planner is able to find solution, mercury_plan.1 file will be created

under the following location:

        ~/.ros/mercury_plan.1

A. To test manually the component:

        roslaunch mcr_task_planners mercury_planner.launch

        rostopic pub /mercury_planner/event_in std_msgs/String "data: 'e_trigger'"
        rostopic echo /mercury_planner/event_out

expected outcome on rostopic echo .. terminal :

        data: e_success
