#!/usr/bin/env python

def convert_IPC2014_plan_file_to_list(plan_path):
    '''
    Load a plan file (in IPC2014 format) and convert into a list
    input: the full path of the plan file
    output: a list of actions (a plan in a list form)
    '''
    # open file in read mode
    try:
        plan = open(plan_path, 'r')
    except:
        args_array = None
        return
    lines = []
    line = plan.readline()
    lines.append(line)
    while line!="":
        line = plan.readline()
        lines.append(line)
    # remove the last element
    lines.pop()
    args_array = []
    for each in lines:
        args = each.split(' ')
        first = args[0]
        last = args[len(args)-1]
        args[0] = first[1:]
        args[len(args)-1] = last[:-2]
        args_array.append(args)
    # close file
    plan.close()
    return args_array
