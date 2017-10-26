#!/bin/bash

# usage
echo "usage: ./plan_with_ffha.sh robot domain problem"

# the plan to use (executable)
planner="rosrun ffha ffha"

# pddl inputs to the planner
domain="${1}/pddl/${2}/domain.pddl"

# Plan!
$planner -o $domain -f ${1}/pddl/${2}/problems/${3}.pddl
