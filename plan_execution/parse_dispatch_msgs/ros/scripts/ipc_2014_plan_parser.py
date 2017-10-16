#!/usr/bin/env python
import os

def ParseMercuryOutput():
    plan = open(os.getenv("HOME") + '/.ros/mercury.plan', 'r')
    lines = []
    line = plan.readline()
    lines.append(line)
    while line!="":
        line = plan.readline()
        lines.append(line)
    #remove the last element
    lines.pop()
    args_array = []
    for each in lines:
        args = each.split(' ')
        first = args[0]
        last = args[len(args)-1]
        args[0] = first[1:]
        args[len(args)-1] = last[:-2]
        args_array.append(args)
    #close file
    plan.close()
    return args_array
    
def test_this_script():
    print ParseMercuryOutput()
    
if __name__ == "__main__":
    test_this_script()
